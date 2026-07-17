"""
Motor control realtime plotter — Qt dashboard for V1 serial telemetry.

Parses ControlPacket (0x55 @ 500 Hz), StatusPacket (0xAA @ 10 Hz), and framed
Debug messages (0xFF 0x04 0x06 + newline-terminated text) per
docs/serial_protocol.md / src/debug/Debug.hpp. Plots motor X/Y target vs
mechanical angles over time. The host computes sine-wave setpoints and streams
them via TARGET_X_SET / TARGET_Y_SET (0x10 / 0x14). Firmware ScanPath is not used.

Flash the Teensy plotter stub first:
    pio run -e teensy41 --target upload
    # entry: src/_teensy_main_stub.cpp

Usage:
    pip install PySide6 pyqtgraph pyserial
    python realtime_plotter.py
"""

from __future__ import annotations

import math
import struct
import sys
import threading
import time
from collections import deque
from itertools import islice
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets

# ---------- Protocol (mirrors Interface.hpp / Debug.hpp) ----------
CONTROL_HEADER = 0x55
STATUS_HEADER = 0xAA
# Debug::msg — Serial.write(DEBUG_HEADER) then Serial.println(text)
DEBUG_HEADER = bytes((0xFF, 0x04, 0x06))
DEBUG_HEADER_SIZE = len(DEBUG_HEADER)
CONTROL_STRUCT = struct.Struct("<BffHff")  # 19 B — matches Interface.hpp ControlPacket
STATUS_STRUCT = struct.Struct("<BBBBffffffffffff")
CONTROL_SIZE = CONTROL_STRUCT.size
STATUS_SIZE = STATUS_STRUCT.size

DISABLE_ENABLE_CMD = 0x01
TOGGLE_CONTROL_MODE_CMD = 0x03
VELOCITY_P_SET_CMD = 0x08
VELOCITY_I_SET_CMD = 0x09
VELOCITY_D_SET_CMD = 0x0A
VELOCITY_RAMP_SET_CMD = 0x0B
ANGLE_P_SET_CMD = 0x0C
ANGLE_I_SET_CMD = 0x0D
ANGLE_D_SET_CMD = 0x0E
ANGLE_RAMP_SET_CMD = 0x0F
TARGET_X_SET_CMD = 0x10  # Interface.hpp TARGET_X_SET_CMD
TARGET_Y_SET_CMD = 0x14  # Interface.hpp TARGET_Y_SET_CMD
FOC_VOLTAGE_LPF_TAU_SET_CMD = 0x11
VELOCITY_LPF_TAU_SET_CMD = 0x12
ANGLE_LPF_TAU_SET_CMD = 0x13
VELOCITY_FF_SET_CMD = 0x92

DEFAULT_BAUD = 460800
HISTORY_SAMPLES = 15000  # ~30 s at 500 Hz
TWO_PI = 2.0 * math.pi
PATH_UPDATE_PERIOD_S = 0.1  # host sine step period (matches UI label)
PLOT_INTERVAL_MS = 33  # ~30 Hz

CONTROL_MODE_LABELS = {0: "PATH", 1: "ANGLE", 2: "VELOCITY"}


def normalize_angle(angle: float) -> float:
    """Mirror firmware normalise_angle() — wrap radians into [0, 2π)."""
    a = math.fmod(angle, TWO_PI)
    return a if a >= 0.0 else (a + TWO_PI)

ANGLE_CURVES = (
    ("X target", 0, "#4da6ff", QtCore.Qt.PenStyle.SolidLine),
    ("X mechanical", 2, "#4da6ff", QtCore.Qt.PenStyle.DashLine),
    ("Y target", 1, "#ff9944", QtCore.Qt.PenStyle.SolidLine),
    ("Y mechanical", 3, "#ff9944", QtCore.Qt.PenStyle.DashLine),
)

# Indices into ControlPacket fields after header unpack:
# (header, target_x, target_y, distance, mechanical_x, mechanical_y)
_CONTROL_TARGET_X = 1
_CONTROL_TARGET_Y = 2
_CONTROL_MECH_X = 4
_CONTROL_MECH_Y = 5

TUNING_FIELDS = (
    ("velocity_p", VELOCITY_P_SET_CMD),
    ("velocity_i", VELOCITY_I_SET_CMD),
    ("velocity_d", VELOCITY_D_SET_CMD),
    ("velocity_ramp", VELOCITY_RAMP_SET_CMD),
    ("angle_p", ANGLE_P_SET_CMD),
    ("angle_i", ANGLE_I_SET_CMD),
    ("angle_d", ANGLE_D_SET_CMD),
    ("angle_ramp", ANGLE_RAMP_SET_CMD),
    ("velocity_lpf_tau", VELOCITY_LPF_TAU_SET_CMD),
    ("angle_lpf_tau", ANGLE_LPF_TAU_SET_CMD),
    ("foc_voltage_lpf_tau", FOC_VOLTAGE_LPF_TAU_SET_CMD),
    ("velocity_feed_forward", VELOCITY_FF_SET_CMD),
)

STATUS_DEFAULTS = {name: 0.0 for name, _ in TUNING_FIELDS}
STATUS_DEFAULTS.update(
    {
        "velocity_p": 1.0,
        "angle_p": 1.0,
        "angle_i": 0.5,
        "velocity_lpf_tau": 0.2,
    }
)


@dataclass
class SineWaveParams:
    """y = a·π·(sin(b·x) + 1) + c; x advances by PATH_UPDATE_PERIOD_S each step."""

    a: float = 0.5
    b: float = 0.001
    c: float = 0.02


class HostSineWavePath:
    """Host-side sine path: X is phase/time, Y follows the parametric equation."""

    def __init__(self, params: SineWaveParams | None = None):
        self._lock = threading.Lock()
        self.params = params or SineWaveParams()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._x = 0.0
            self.theta = 0.0
            self.phi = 0.0

    def set_params(self, params: SineWaveParams) -> None:
        with self._lock:
            self.params = params

    def step(self) -> tuple[float, float]:
        with self._lock:
            p = self.params
            self._x += PATH_UPDATE_PERIOD_S
            y = p.a * math.pi * (math.sin(p.b * self._x) + 1.0) + p.c
            self.theta = normalize_angle(self._x)
            self.phi = normalize_angle(y)
            return self.theta, self.phi


class PathStreamer(threading.Thread):
    """Background thread that streams host path setpoints over serial."""

    def __init__(self, path: HostSineWavePath, write_targets):
        super().__init__(daemon=True, name="PathStreamer")
        self.path = path
        self.write_targets = write_targets
        self._stop_event = threading.Event()
        self._scanning = threading.Event()

    def start_scanning(self) -> None:
        self.path.reset()
        self._scanning.set()

    def stop_scanning(self) -> None:
        self._scanning.clear()

    def shutdown(self) -> None:
        self._stop_event.set()
        self._scanning.clear()

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self._scanning.is_set():
                theta, phi = self.path.step()
                self.write_targets(theta, phi)
                if self._stop_event.wait(PATH_UPDATE_PERIOD_S):
                    break
            elif self._stop_event.wait(0.05):
                break


class AngleHistory:
    def __init__(self):
        self.lock = threading.Lock()
        self.times = deque(maxlen=HISTORY_SAMPLES)
        self.samples = deque(maxlen=HISTORY_SAMPLES)
        self.control_count = 0

    def append(self, timestamp: float, packet: tuple) -> None:
        with self.lock:
            self.times.append(timestamp)
            self.samples.append(
                (
                    normalize_angle(float(packet[_CONTROL_TARGET_X])),
                    normalize_angle(float(packet[_CONTROL_TARGET_Y])),
                    normalize_angle(float(packet[_CONTROL_MECH_X])),
                    normalize_angle(float(packet[_CONTROL_MECH_Y])),
                )
            )
            self.control_count += 1

    def clear(self) -> None:
        with self.lock:
            self.times.clear()
            self.samples.clear()

    def snapshot(self, now: float, window: float) -> tuple[np.ndarray, list[np.ndarray]]:
        """Copy only samples within the visible time window (relative to now)."""
        cutoff = now - window
        with self.lock:
            n = len(self.times)
            if n == 0:
                return np.empty(0, dtype=np.float64), []

            start = n
            for i, ts in enumerate(self.times):
                if ts >= cutoff:
                    start = i
                    break

            count = n - start
            if count == 0:
                return np.empty(0, dtype=np.float64), []

            rel_t = np.fromiter(
                (ts - now for ts in islice(self.times, start, None)),
                dtype=np.float64,
                count=count,
            )
            rows = list(islice(self.samples, start, None))

        arr = np.asarray(rows, dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        return rel_t, [arr[:, col] for _, col, _, _ in ANGLE_CURVES]


class SerialReader(threading.Thread):
    def __init__(
        self, port: str, baud: int, on_control, on_status, on_text, on_debug, on_error
    ):
        super().__init__(daemon=True)
        self.port_name = port
        self.baud = baud
        self.on_control = on_control
        self.on_status = on_status
        self.on_text = on_text
        self.on_debug = on_debug
        self.on_error = on_error
        self._stop_event = threading.Event()
        self._serial: serial.Serial | None = None
        self._rx_buffer = bytearray()
        self._text_buffer = bytearray()
        self._write_lock = threading.Lock()

    def stop(self) -> None:
        self._stop_event.set()

    def write_command(self, cmd: int, value: float = 0.0) -> None:
        ser = self._serial
        if ser is not None and ser.is_open:
            try:
                with self._write_lock:
                    ser.write(struct.pack("<Bf", cmd, float(value)))
            except serial.SerialException:
                pass

    def run(self) -> None:
        try:
            self._serial = serial.Serial(self.port_name, self.baud, timeout=0.05)
        except serial.SerialException as exc:
            self.on_error(f"Failed to open {self.port_name}: {exc}")
            return

        try:
            while not self._stop_event.is_set():
                try:
                    chunk = self._serial.read(4096)
                except serial.SerialException as exc:
                    self.on_error(f"Serial error: {exc}")
                    break
                if chunk:
                    self._rx_buffer.extend(chunk)
                    self._parse_buffer()
        finally:
            try:
                if self._serial is not None:
                    self._serial.close()
            except serial.SerialException:
                pass

    @staticmethod
    def _find_next_frame(buf: bytearray, start: int = 1) -> int:
        """Index of the next control/status/debug frame, or -1 if none."""
        next_hdr = -1
        for h in (CONTROL_HEADER, STATUS_HEADER):
            idx = buf.find(bytes([h]), start)
            if idx >= 0 and (next_hdr < 0 or idx < next_hdr):
                next_hdr = idx
        idx = buf.find(DEBUG_HEADER, start)
        if idx >= 0 and (next_hdr < 0 or idx < next_hdr):
            next_hdr = idx
        return next_hdr

    def _parse_buffer(self) -> None:
        buf = self._rx_buffer
        while buf:
            header = buf[0]
            if header == CONTROL_HEADER:
                if len(buf) < CONTROL_SIZE:
                    break
                packet = CONTROL_STRUCT.unpack_from(buf, 0)
                del buf[:CONTROL_SIZE]
                self.on_control(time.monotonic(), packet)
                continue
            if header == STATUS_HEADER:
                if len(buf) < STATUS_SIZE:
                    break
                packet = STATUS_STRUCT.unpack_from(buf, 0)
                del buf[:STATUS_SIZE]
                self.on_status(packet)
                continue
            if header == DEBUG_HEADER[0]:
                if len(buf) < DEBUG_HEADER_SIZE:
                    break
                if buf[:DEBUG_HEADER_SIZE] == DEBUG_HEADER:
                    nl = buf.find(b"\n", DEBUG_HEADER_SIZE)
                    if nl < 0:
                        if len(buf) > 512:
                            del buf[0]
                            continue
                        break
                    line = bytes(buf[DEBUG_HEADER_SIZE : nl + 1])
                    del buf[: nl + 1]
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        self.on_debug(text)
                    continue
                # Lone 0xFF that is not a debug frame — treat as text/resync below.

            nl = buf.find(b"\n")
            next_hdr = self._find_next_frame(buf, 1)
            if nl >= 0 and (next_hdr < 0 or nl < next_hdr):
                line = bytes(buf[: nl + 1])
                del buf[: nl + 1]
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    self.on_text(text)
            elif next_hdr > 0:
                self._text_buffer.extend(buf[:next_hdr])
                del buf[:next_hdr]
                self._flush_text()
            elif next_hdr == 0:
                del buf[:1]
            elif len(buf) > 512:
                self._text_buffer.extend(buf)
                buf.clear()
                self._flush_text()
            else:
                break
        self._flush_text()

    def _flush_text(self) -> None:
        while True:
            idx = self._text_buffer.find(b"\n")
            if idx < 0:
                break
            line = self._text_buffer[: idx + 1]
            del self._text_buffer[: idx + 1]
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                self.on_text(text)
        if len(self._text_buffer) > 4096:
            self._text_buffer.clear()


class RealtimePlotter(QtWidgets.QMainWindow):
    text_received = QtCore.Signal(str)
    debug_received = QtCore.Signal(str)
    error_received = QtCore.Signal(str)
    status_received = QtCore.Signal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control Realtime Plotter")
        self.resize(1400, 860)

        self.reader: SerialReader | None = None
        self.history = AngleHistory()
        self.path = HostSineWavePath()
        self._tuning_values = dict(STATUS_DEFAULTS)
        self.paused = False
        self._enabled = False
        self._scanning = False
        self._control_mode = 1
        self._rate_last_time = time.monotonic()
        self._rate_last_count = 0

        self.text_received.connect(self._append_log)
        self.debug_received.connect(self._append_debug)
        self.error_received.connect(self._on_serial_error)
        self.status_received.connect(self._on_status_ui)

        self._tuning_spins: dict[str, QtWidgets.QDoubleSpinBox] = {}
        self._path_spins: dict[str, QtWidgets.QWidget] = {}

        self._build_ui()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(PLOT_INTERVAL_MS)

        self.rate_timer = QtCore.QTimer(self)
        self.rate_timer.timeout.connect(self._update_rate)
        self.rate_timer.start(1000)

        self.path_streamer = PathStreamer(self.path, self._write_targets)
        self.path_streamer.start()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMaximumWidth(380)
        panel_widget = QtWidgets.QWidget()
        panel = QtWidgets.QVBoxLayout(panel_widget)
        panel.setSpacing(8)
        scroll.setWidget(panel_widget)
        root.addWidget(scroll, 0)

        conn = QtWidgets.QGroupBox("Connection")
        conn_layout = QtWidgets.QGridLayout(conn)
        self.port_combo = QtWidgets.QComboBox()
        refresh_btn = QtWidgets.QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        self.baud_edit = QtWidgets.QLineEdit(str(DEFAULT_BAUD))
        self.baud_edit.setMaximumWidth(90)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.link_label = QtWidgets.QLabel("Disconnected")
        self.link_label.setStyleSheet("color: #ff5555;")
        conn_layout.addWidget(QtWidgets.QLabel("Port:"), 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(refresh_btn, 0, 2)
        conn_layout.addWidget(QtWidgets.QLabel("Baud:"), 1, 0)
        conn_layout.addWidget(self.baud_edit, 1, 1)
        conn_layout.addWidget(self.connect_btn, 1, 2)
        conn_layout.addWidget(self.link_label, 2, 0, 1, 3)
        panel.addWidget(conn)

        state = QtWidgets.QGroupBox("System State")
        state_layout = QtWidgets.QVBoxLayout(state)
        self.enable_btn = QtWidgets.QPushButton("Enable")
        self.enable_btn.clicked.connect(self._send_enable_toggle)
        self._style_enable_btn()
        self.mode_label = QtWidgets.QLabel("Mode: —")
        self.mode_btn = QtWidgets.QPushButton("Toggle Control Mode")
        self.mode_btn.clicked.connect(lambda: self._send_cmd(TOGGLE_CONTROL_MODE_CMD))
        state_layout.addWidget(self.enable_btn)
        state_layout.addWidget(self.mode_label)
        state_layout.addWidget(self.mode_btn)
        panel.addWidget(state)

        manual_box = QtWidgets.QGroupBox("Manual targets")
        manual_layout = QtWidgets.QFormLayout(manual_box)
        self.manual_x_spin = QtWidgets.QDoubleSpinBox()
        self.manual_x_spin.setDecimals(6)
        self.manual_x_spin.setRange(-10.0, 10.0)
        self.manual_x_spin.setValue(0.0)
        self.manual_y_spin = QtWidgets.QDoubleSpinBox()
        self.manual_y_spin.setDecimals(6)
        self.manual_y_spin.setRange(-10.0, 10.0)
        self.manual_y_spin.setValue(0.0)
        manual_layout.addRow("X target [rad]:", self.manual_x_spin)
        manual_layout.addRow("Y target [rad]:", self.manual_y_spin)
        set_targets_btn = QtWidgets.QPushButton("Set targets")
        set_targets_btn.clicked.connect(self._send_manual_targets)
        manual_layout.addRow(set_targets_btn)
        manual_note = QtWidgets.QLabel("Sending manual targets stops scanning.")
        manual_note.setWordWrap(True)
        manual_note.setStyleSheet("color: #888;")
        manual_layout.addRow(manual_note)
        panel.addWidget(manual_box)

        path_box = QtWidgets.QGroupBox("Sine wave path (host-generated)")
        path_layout = QtWidgets.QFormLayout(path_box)
        path_note = QtWidgets.QLabel(
            "y = a·π·(sin(b·x) + 1) + c; x advances each "
            f"{PATH_UPDATE_PERIOD_S:.1f} s. "
            "X target = x, Y target = y (both normalised to [0, 2π)). "
            "Setpoints are sent as TARGET_X/Y (0x10/0x14)."
        )
        path_note.setWordWrap(True)
        path_note.setStyleSheet("color: #888;")
        path_layout.addRow(path_note)

        self.scan_btn = QtWidgets.QPushButton("Not scanning")
        self.scan_btn.clicked.connect(self._toggle_scanning)
        self._style_scan_btn()
        path_layout.addRow(self.scan_btn)

        self._add_path_spin(path_layout, "a", "a")
        self._add_path_spin(path_layout, "b", "b")
        self._add_path_spin(path_layout, "c [rad]", "c")
        path_layout.addRow(
            "Update period:",
            QtWidgets.QLabel(f"{PATH_UPDATE_PERIOD_S:.1f} s"),
        )
        panel.addWidget(path_box)

        tuning_box = QtWidgets.QGroupBox("Tuning")
        tuning_layout = QtWidgets.QFormLayout(tuning_box)
        labels = {
            "velocity_p": "Velocity P",
            "velocity_i": "Velocity I",
            "velocity_d": "Velocity D",
            "velocity_ramp": "Velocity ramp",
            "angle_p": "Angle P",
            "angle_i": "Angle I",
            "angle_d": "Angle D",
            "angle_ramp": "Angle ramp",
            "velocity_lpf_tau": "Velocity LPF τ [s]",
            "angle_lpf_tau": "Angle LPF τ [s]",
            "foc_voltage_lpf_tau": "FOC voltage LPF τ [s]",
            "velocity_feed_forward": "Velocity FF",
        }
        for field, cmd in TUNING_FIELDS:
            row = QtWidgets.QHBoxLayout()
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(6)
            spin.setRange(-1e6, 1e6)
            spin.setValue(self._tuning_values[field])
            btn = QtWidgets.QPushButton("Set")
            btn.setFixedWidth(48)
            btn.clicked.connect(
                lambda _=False, f=field, c=cmd, s=spin: self._send_tuning(f, c, s)
            )
            row.addWidget(spin, 1)
            row.addWidget(btn)
            wrap = QtWidgets.QWidget()
            wrap.setLayout(row)
            tuning_layout.addRow(labels[field] + ":", wrap)
            self._tuning_spins[field] = spin
        panel.addWidget(tuning_box)

        disp = QtWidgets.QGroupBox("Display")
        disp_layout = QtWidgets.QGridLayout(disp)
        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(0.5, 120.0)
        self.window_spin.setValue(10.0)
        self.window_spin.setSingleStep(1.0)
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._toggle_pause)
        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(self.history.clear)
        disp_layout.addWidget(QtWidgets.QLabel("Window [s]:"), 0, 0)
        disp_layout.addWidget(self.window_spin, 0, 1)
        disp_layout.addWidget(self.pause_btn, 1, 0)
        disp_layout.addWidget(clear_btn, 1, 1)
        panel.addWidget(disp)

        self.rate_label = QtWidgets.QLabel("0 control pkt/s")
        panel.addWidget(self.rate_label)
        panel.addStretch(1)

        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        pg.setConfigOptions(antialias=False)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#101014")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "time", units="s")
        self.plot_widget.setLabel("left", "angle", units="rad")
        self.plot_widget.setYRange(0.0, TWO_PI, padding=0.02)
        self.plot_widget.addLegend(offset=(10, 10))
        self.curves = []
        for label, _, colour, style in ANGLE_CURVES:
            pen = pg.mkPen(colour, width=1.5, style=style)
            self.curves.append(self.plot_widget.plot([], [], pen=pen, name=label))
        right.addWidget(self.plot_widget, 1)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(500)
        self.log_box.setFixedHeight(160)
        self.log_box.setPlaceholderText("Serial text / Debug::msg output…")
        self.log_box.setStyleSheet(
            "background-color: #101014; color: #cccccc; font-family: Consolas, monospace;"
        )
        right.addWidget(self.log_box, 0)

        self._refresh_ports()
        self._apply_path_params()
        self._update_mode_label()

    def _add_path_spin(
        self,
        layout: QtWidgets.QFormLayout,
        label: str,
        key: str,
    ) -> None:
        defaults = SineWaveParams()
        spin = QtWidgets.QDoubleSpinBox()
        spin.setDecimals(6)
        spin.setRange(-100.0, 100.0)
        spin.setValue(float(getattr(defaults, key)))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(spin, 1)
        apply_btn = QtWidgets.QPushButton("Set")
        apply_btn.setFixedWidth(48)
        apply_btn.clicked.connect(self._apply_path_params)
        row.addWidget(apply_btn)
        wrap = QtWidgets.QWidget()
        wrap.setLayout(row)
        layout.addRow(label + ":", wrap)
        self._path_spins[key] = spin

    def _read_path_params(self) -> SineWaveParams:
        return SineWaveParams(
            a=float(self._path_spins["a"].value()),
            b=float(self._path_spins["b"].value()),
            c=float(self._path_spins["c"].value()),
        )

    def _apply_path_params(self) -> None:
        self.path.set_params(self._read_path_params())

    def _style_scan_btn(self) -> None:
        if self._scanning:
            self.scan_btn.setText("Scanning — click to stop")
            self.scan_btn.setStyleSheet(
                "background-color: #1f4f6b; color: white; font-weight: bold;"
            )
        else:
            self.scan_btn.setText("Not scanning — click to start")
            self.scan_btn.setStyleSheet(
                "background-color: #3a3a3a; color: #cccccc;"
            )

    def _set_scanning(self, scanning: bool) -> None:
        if scanning == self._scanning:
            return
        self._scanning = scanning
        if scanning:
            self._apply_path_params()
            self.path_streamer.start_scanning()
        else:
            self.path_streamer.stop_scanning()
        self._style_scan_btn()
        self._update_mode_label()

    def _stop_scanning(self) -> None:
        self._set_scanning(False)

    def _toggle_scanning(self) -> None:
        self._set_scanning(not self._scanning)

    def _send_manual_targets(self) -> None:
        self._stop_scanning()
        theta = normalize_angle(self.manual_x_spin.value())
        phi = normalize_angle(self.manual_y_spin.value())
        self._send_cmd(TARGET_X_SET_CMD, theta)
        self._send_cmd(TARGET_Y_SET_CMD, phi)

    def _write_targets(self, theta: float, phi: float) -> None:
        reader = self.reader
        if reader is None:
            return
        reader.write_command(TARGET_X_SET_CMD, theta)
        reader.write_command(TARGET_Y_SET_CMD, phi)

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _toggle_connection(self) -> None:
        if self.reader is not None:
            self._disconnect()
            return
        port = self.port_combo.currentText()
        if not port:
            self._append_log("No serial port selected")
            return
        try:
            baud = int(self.baud_edit.text())
        except ValueError:
            self._append_log("Invalid baud rate")
            return
        self.reader = SerialReader(
            port,
            baud,
            on_control=self._on_control_thread,
            on_status=self._on_status_thread,
            on_text=self.text_received.emit,
            on_debug=self.debug_received.emit,
            on_error=self.error_received.emit,
        )
        self.reader.start()
        self.connect_btn.setText("Disconnect")
        self.link_label.setText(f"Connected: {port} @ {baud}")
        self.link_label.setStyleSheet("color: #55ff55;")
        self._append_log(f"Connected to {port} @ {baud}")

    def _disconnect(self) -> None:
        self._stop_scanning()
        if self.reader is not None:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.connect_btn.setText("Connect")
        self.link_label.setText("Disconnected")
        self.link_label.setStyleSheet("color: #ff5555;")
        self._append_log("Disconnected")

    def _on_serial_error(self, message: str) -> None:
        self._append_log(message)
        self._disconnect()

    def _send_cmd(self, cmd: int, value: float = 0.0) -> None:
        if self.reader is None:
            self._append_log("Not connected")
            return
        self.reader.write_command(cmd, value)

    def _send_tuning(
        self, field: str, cmd: int, spin: QtWidgets.QDoubleSpinBox
    ) -> None:
        value = spin.value()
        self._tuning_values[field] = value
        self._send_cmd(cmd, value)

    def _send_enable_toggle(self) -> None:
        self._send_cmd(DISABLE_ENABLE_CMD)
        print("ENABLE CMD SENT")

    def _style_enable_btn(self) -> None:
        if self._enabled:
            self.enable_btn.setText("Enabled — click to disable")
            self.enable_btn.setStyleSheet(
                "background-color: #1f6b3a; color: white; font-weight: bold;"
            )
        else:
            self.enable_btn.setText("Disabled — click to enable")
            self.enable_btn.setStyleSheet(
                "background-color: #6b1f1f; color: white; font-weight: bold;"
            )

    def _on_control_thread(self, timestamp: float, packet: tuple) -> None:
        self.history.append(timestamp, packet)

    def _on_status_thread(self, packet: tuple) -> None:
        self.status_received.emit(packet)

    def _update_mode_label(self) -> None:
        mode_name = CONTROL_MODE_LABELS.get(self._control_mode, str(self._control_mode))
        scan_state = "scanning" if self._scanning else "not scanning"
        self.mode_label.setText(f"Mode: {mode_name} · {scan_state}")

    def _on_status_ui(self, packet: tuple) -> None:
        enabled = bool(packet[1])
        control_mode = int(packet[2])
        if enabled != self._enabled:
            self._enabled = enabled
            self._style_enable_btn()
        self._control_mode = control_mode
        self._update_mode_label()

    def _toggle_pause(self, checked: bool) -> None:
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    def _update_plot(self) -> None:
        if self.paused:
            return
        now = time.monotonic()
        window = self.window_spin.value()
        rel_t, channels = self.history.snapshot(now, window)
        if rel_t.size == 0:
            return
        for curve, ch in zip(self.curves, channels):
            curve.setData(rel_t, ch)
        self.plot_widget.setXRange(-window, 0, padding=0)

    def _update_rate(self) -> None:
        now = time.monotonic()
        dt = now - self._rate_last_time
        count = self.history.control_count
        rate = max(0.0, (count - self._rate_last_count) / dt) if dt > 0 else 0.0
        self._rate_last_count = count
        self._rate_last_time = now
        self.rate_label.setText(f"{rate:.0f} control pkt/s")

    def _append_log(self, text: str) -> None:
        cursor = self.log_box.textCursor()
        cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
        fmt = QtGui.QTextCharFormat()
        fmt.setForeground(QtGui.QColor("#cccccc"))
        cursor.insertText(text + "\n", fmt)
        self.log_box.setTextCursor(cursor)
        self.log_box.ensureCursorVisible()

    def _append_debug(self, text: str) -> None:
        cursor = self.log_box.textCursor()
        cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
        fmt = QtGui.QTextCharFormat()
        fmt.setForeground(QtGui.QColor("#ffcc66"))
        cursor.insertText(f"[DEBUG] {text}\n", fmt)
        self.log_box.setTextCursor(cursor)
        self.log_box.ensureCursorVisible()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._disconnect()
        self.path_streamer.shutdown()
        self.path_streamer.join(timeout=1.0)
        super().closeEvent(event)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    window = RealtimePlotter()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
