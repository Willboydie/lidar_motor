"""
Motor control realtime plotter — Qt dashboard for V1 serial telemetry.

Parses ControlPacket (0x55 @ CONTROL_PACKET_PERIOD_US, 27 B), StatusPacket
(0xAA @ 10 Hz), and framed Debug messages (0xFF 0x04 0x06 + newline-terminated
text) per docs/serial_protocol.md / src/debug/Debug.hpp.

Primary use with scan_engine_test: XY plot of FOC target angles
(ControlPacket.target_angle_x vs target_angle_y) — last N samples at once.
Mechanical trail is shown dashed when available; associated angles ignored.

Host → device commands use framed payloads matching src/parser/Parser.hpp:
    [cmd:1][len_lo:1][len_hi:1][payload: len bytes]
Interface.onCommand enacts them (ScanEngine enable / coarse / P2P / tuning).
See src/utils/cmd_defs.h.

Flash the ScanEngine bench first:
    pio run -e teensy41 --target upload
    # entry: src/_teensy_main_stub.cpp → tests/scan/scan_engine_test.hpp

Typical flow to see targets move:
    Connect → Scan Engine Enable → Coarse Begin
    (System Enable is optional; scan_engine_test streams targets without FOC loops.)

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

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets

# ---------- Protocol (mirrors Interface.hpp / cmd_defs.h / Debug.hpp) ----------
CONTROL_HEADER = 0x55
STATUS_HEADER = 0xAA
# Debug::msg — Serial.write(DEBUG_HEADER) then Serial.println(text)
DEBUG_HEADER = bytes((0xFF, 0x04, 0x06))
DEBUG_HEADER_SIZE = len(DEBUG_HEADER)
CONTROL_STRUCT = struct.Struct("<BffHffff")  # 27 B — matches Interface.hpp ControlPacket
STATUS_STRUCT = struct.Struct("<BBBBBfffffffffffff")  # 57 B — matches Interface.hpp StatusPacket
CONTROL_SIZE = CONTROL_STRUCT.size
STATUS_SIZE = STATUS_STRUCT.size

DISABLE_ENABLE_CMD = 0x01
SET_DEBUG_MODE_CMD = 0x02
VELOCITY_P_SET_CMD = 0x08
VELOCITY_I_SET_CMD = 0x09
ANGLE_P_SET_CMD = 0x0C
ANGLE_I_SET_CMD = 0x0D
VELOCITY_LPF_TAU_SET_CMD = 0x12

# Scan Engine (src/utils/cmd_defs.h)
ENABLE_SCANENGINE_CMD = 0x1D
DISABLE_SCANENGINE_CMD = 0x1E
BEGIN_COARSE_SCANNING_CMD = 0x1F
X_AMPLITUDE_SET_CMD = 0x21
Y_AMPLITUDE_SET_CMD = 0x22
X_FREQUENCY_SET_CMD = 0x23
Y_FREQUENCY_SET_CMD = 0x24
X_PHASE_SET_CMD = 0x25
Y_PHASE_SET_CMD = 0x26
STOP_COARSE_SCANNING_CMD = 0x27
BEGIN_POINT2POINT_SCANNING_CMD = 0x28
STOP_POINT2POINT_SCANNING_CMD = 0x29
SET_POINT2POINT_TARGET_CMD = 0x2A
UPDATE_COARSE_SCAN_CENTRE_CMD = 0x2B

DEFAULT_BAUD = 460800
N_POINTS = 5 * 250  # points shown at once on the XY plot
HISTORY_SAMPLES = N_POINTS
TWO_PI = 2.0 * math.pi
PLOT_INTERVAL_MS = 33  # ~30 Hz UI refresh


def normalize_angle(angle: float) -> float:
    """Mirror firmware normalise_angle() — wrap radians into [0, 2π)."""
    a = math.fmod(angle, TWO_PI)
    return a if a >= 0.0 else (a + TWO_PI)

# ControlPacket unpack indices (header + fields):
# 0 header, 1 target_x, 2 target_y, 3 distance,
# 4 mechanical_x, 5 mechanical_y, 6 associated_x, 7 associated_y
_CONTROL_TARGET_X = 1
_CONTROL_TARGET_Y = 2
_CONTROL_MECH_X = 4
_CONTROL_MECH_Y = 5

TUNING_FIELDS = (
    ("velocity_p", VELOCITY_P_SET_CMD),
    ("velocity_i", VELOCITY_I_SET_CMD),
    ("angle_p", ANGLE_P_SET_CMD),
    ("angle_i", ANGLE_I_SET_CMD),
    ("velocity_lpf_tau", VELOCITY_LPF_TAU_SET_CMD),
)

STATUS_DEFAULTS = {name: 0.0 for name, _ in TUNING_FIELDS}
STATUS_DEFAULTS.update(
    {
        "velocity_p": 1.0,
        "angle_p": 6.0,
        "angle_i": 12.0,
        "velocity_lpf_tau": 0.2,
    }
)

COARSE_PARAM_FIELDS = (
    ("x_amplitude", X_AMPLITUDE_SET_CMD, 0.2),
    ("y_amplitude", Y_AMPLITUDE_SET_CMD, 0.2),
    ("x_frequency", X_FREQUENCY_SET_CMD, 1.5),
    ("y_frequency", Y_FREQUENCY_SET_CMD, 3.0),
    ("x_phase", X_PHASE_SET_CMD, 0.0),
    ("y_phase", Y_PHASE_SET_CMD, 0.0),
)


class AngleHistory:
    def __init__(self, maxlen: int = HISTORY_SAMPLES):
        self.lock = threading.Lock()
        self.maxlen = maxlen
        self.target_xy = deque(maxlen=maxlen)  # (angle_x, angle_y)
        self.mech_xy = deque(maxlen=maxlen)
        self.control_count = 0

    def set_maxlen(self, n: int) -> None:
        n = max(1, int(n))
        with self.lock:
            if n == self.maxlen:
                return
            self.maxlen = n
            self.target_xy = deque(self.target_xy, maxlen=n)
            self.mech_xy = deque(self.mech_xy, maxlen=n)

    def append(self, timestamp: float, packet: tuple) -> None:
        del timestamp  # XY trail is sample-count based, not time-windowed
        with self.lock:
            self.target_xy.append(
                (
                    normalize_angle(float(packet[_CONTROL_TARGET_X])),
                    normalize_angle(float(packet[_CONTROL_TARGET_Y])),
                )
            )
            self.mech_xy.append(
                (
                    normalize_angle(float(packet[_CONTROL_MECH_X])),
                    normalize_angle(float(packet[_CONTROL_MECH_Y])),
                )
            )
            self.control_count += 1

    def clear(self) -> None:
        with self.lock:
            self.target_xy.clear()
            self.mech_xy.clear()

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return last N points as (target_x, target_y, mech_x, mech_y)."""
        with self.lock:
            if not self.target_xy:
                empty = np.empty(0, dtype=np.float64)
                return empty, empty, empty, empty
            target = np.asarray(self.target_xy, dtype=np.float64)
            mech = np.asarray(self.mech_xy, dtype=np.float64)
        return target[:, 0], target[:, 1], mech[:, 0], mech[:, 1]


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

    def write_command(self, cmd: int, *values: float) -> None:
        """Send [cmd][len_lo][len_hi][payload floats…] — matches Parser framing."""
        ser = self._serial
        if ser is not None and ser.is_open:
            try:
                payload = b"".join(struct.pack("<f", float(v)) for v in values)
                frame = struct.pack("<BH", cmd & 0xFF, len(payload)) + payload
                with self._write_lock:
                    ser.write(frame)
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
        self._tuning_values = dict(STATUS_DEFAULTS)
        self.paused = False
        self._enabled = False
        self._debug = True  # matches DEFAULT_DEBUG_SETTING
        self._scanning = False
        self._control_mode = 1
        self._rate_last_time = time.monotonic()
        self._rate_last_count = 0

        self.text_received.connect(self._append_log)
        self.debug_received.connect(self._append_debug)
        self.error_received.connect(self._on_serial_error)
        self.status_received.connect(self._on_status_ui)

        self._tuning_spins: dict[str, QtWidgets.QDoubleSpinBox] = {}
        self._coarse_spins: dict[str, QtWidgets.QDoubleSpinBox] = {}

        self._build_ui()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(PLOT_INTERVAL_MS)

        self.rate_timer = QtCore.QTimer(self)
        self.rate_timer.timeout.connect(self._update_rate)
        self.rate_timer.start(1000)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMaximumWidth(500)
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
        self.debug_btn = QtWidgets.QPushButton("Debug")
        self.debug_btn.clicked.connect(self._send_debug_toggle)
        self._style_debug_btn()
        state_layout.addWidget(self.enable_btn)
        state_layout.addWidget(self.debug_btn)
        panel.addWidget(state)

        scan_box = QtWidgets.QGroupBox("Scan Engine")
        scan_layout = QtWidgets.QVBoxLayout(scan_box)
        engine_row = QtWidgets.QHBoxLayout()
        enable_engine_btn = QtWidgets.QPushButton("Enable")
        enable_engine_btn.clicked.connect(
            lambda: self._send_cmd(ENABLE_SCANENGINE_CMD)
        )
        disable_engine_btn = QtWidgets.QPushButton("Disable")
        disable_engine_btn.clicked.connect(
            lambda: self._send_cmd(DISABLE_SCANENGINE_CMD)
        )
        engine_row.addWidget(enable_engine_btn)
        engine_row.addWidget(disable_engine_btn)
        scan_layout.addLayout(engine_row)

        coarse_box = QtWidgets.QGroupBox("Coarse scan")
        coarse_layout = QtWidgets.QFormLayout(coarse_box)
        coarse_btns = QtWidgets.QHBoxLayout()
        begin_coarse_btn = QtWidgets.QPushButton("Begin")
        begin_coarse_btn.clicked.connect(
            lambda: self._send_cmd(BEGIN_COARSE_SCANNING_CMD)
        )
        stop_coarse_btn = QtWidgets.QPushButton("Stop")
        stop_coarse_btn.clicked.connect(
            lambda: self._send_cmd(STOP_COARSE_SCANNING_CMD)
        )
        coarse_btns.addWidget(begin_coarse_btn)
        coarse_btns.addWidget(stop_coarse_btn)
        coarse_layout.addRow(coarse_btns)

        labels = {
            "x_amplitude": "X amplitude [rad]",
            "y_amplitude": "Y amplitude [rad]",
            "x_frequency": "X frequency [Hz]",
            "y_frequency": "Y frequency [Hz]",
            "x_phase": "X phase [rad]",
            "y_phase": "Y phase [rad]",
        }
        for field, cmd, default in COARSE_PARAM_FIELDS:
            self._add_set_spin(coarse_layout, labels[field], field, cmd, default, self._coarse_spins)

        self.centre_x_spin = QtWidgets.QDoubleSpinBox()
        self.centre_x_spin.setDecimals(6)
        self.centre_x_spin.setRange(-10.0, 10.0)
        self.centre_x_spin.setValue(3.54)
        self.centre_y_spin = QtWidgets.QDoubleSpinBox()
        self.centre_y_spin.setDecimals(6)
        self.centre_y_spin.setRange(-10.0, 10.0)
        self.centre_y_spin.setValue(3.39)
        centre_row = QtWidgets.QHBoxLayout()
        centre_row.addWidget(self.centre_x_spin)
        centre_row.addWidget(self.centre_y_spin)
        centre_btn = QtWidgets.QPushButton("Set")
        centre_btn.setFixedWidth(48)
        centre_btn.clicked.connect(self._send_coarse_centre)
        centre_row.addWidget(centre_btn)
        centre_wrap = QtWidgets.QWidget()
        centre_wrap.setLayout(centre_row)
        coarse_layout.addRow("Centre X/Y [rad]:", centre_wrap)
        scan_layout.addWidget(coarse_box)

        p2p_box = QtWidgets.QGroupBox("Point-to-point")
        p2p_layout = QtWidgets.QFormLayout(p2p_box)
        p2p_btns = QtWidgets.QHBoxLayout()
        begin_p2p_btn = QtWidgets.QPushButton("Begin")
        begin_p2p_btn.clicked.connect(
            lambda: self._send_cmd(BEGIN_POINT2POINT_SCANNING_CMD)
        )
        stop_p2p_btn = QtWidgets.QPushButton("Stop")
        stop_p2p_btn.clicked.connect(
            lambda: self._send_cmd(STOP_POINT2POINT_SCANNING_CMD)
        )
        p2p_btns.addWidget(begin_p2p_btn)
        p2p_btns.addWidget(stop_p2p_btn)
        p2p_layout.addRow(p2p_btns)

        self.p2p_x_spin = QtWidgets.QDoubleSpinBox()
        self.p2p_x_spin.setDecimals(6)
        self.p2p_x_spin.setRange(-10.0, 10.0)
        self.p2p_x_spin.setValue(3.54)
        self.p2p_y_spin = QtWidgets.QDoubleSpinBox()
        self.p2p_y_spin.setDecimals(6)
        self.p2p_y_spin.setRange(-10.0, 10.0)
        self.p2p_y_spin.setValue(3.39)
        p2p_row = QtWidgets.QHBoxLayout()
        p2p_row.addWidget(self.p2p_x_spin)
        p2p_row.addWidget(self.p2p_y_spin)
        p2p_set_btn = QtWidgets.QPushButton("Set")
        p2p_set_btn.setFixedWidth(48)
        p2p_set_btn.clicked.connect(self._send_p2p_target)
        p2p_row.addWidget(p2p_set_btn)
        p2p_wrap = QtWidgets.QWidget()
        p2p_wrap.setLayout(p2p_row)
        p2p_layout.addRow("Target X/Y [rad]:", p2p_wrap)
        scan_layout.addWidget(p2p_box)
        panel.addWidget(scan_box)

        tuning_box = QtWidgets.QGroupBox("Tuning")
        tuning_layout = QtWidgets.QFormLayout(tuning_box)
        tuning_labels = {
            "velocity_p": "Velocity P",
            "velocity_i": "Velocity I",
            "angle_p": "Angle P",
            "angle_i": "Angle I",
            "velocity_lpf_tau": "Velocity LPF τ [s]",
        }
        for field, cmd in TUNING_FIELDS:
            self._add_set_spin(
                tuning_layout,
                tuning_labels[field],
                field,
                cmd,
                self._tuning_values[field],
                self._tuning_spins,
            )
        panel.addWidget(tuning_box)

        disp = QtWidgets.QGroupBox("Display")
        disp_layout = QtWidgets.QGridLayout(disp)
        self.n_points_spin = QtWidgets.QSpinBox()
        self.n_points_spin.setRange(10, 100000)
        self.n_points_spin.setValue(N_POINTS)
        self.n_points_spin.setSingleStep(1000)
        self.n_points_spin.valueChanged.connect(self.history.set_maxlen)
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._toggle_pause)
        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(self.history.clear)
        disp_layout.addWidget(QtWidgets.QLabel("N points:"), 0, 0)
        disp_layout.addWidget(self.n_points_spin, 0, 1)
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
        self.plot_widget.setLabel("bottom", "angle X", units="rad")
        self.plot_widget.setLabel("left", "angle Y", units="rad")
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.addLegend(offset=(10, 10))
        self.target_curve = self.plot_widget.plot(
            [],
            [],
            pen=pg.mkPen("#4da6ff", width=1.5),
            name="FOC target",
        )
        self.mech_curve = self.plot_widget.plot(
            [],
            [],
            pen=pg.mkPen("#ff9944", width=1.0, style=QtCore.Qt.PenStyle.DashLine),
            name="mechanical",
        )
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

    def _add_set_spin(
        self,
        layout: QtWidgets.QFormLayout,
        label: str,
        field: str,
        cmd: int,
        default: float,
        store: dict[str, QtWidgets.QDoubleSpinBox],
    ) -> None:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setDecimals(6)
        spin.setRange(-1e6, 1e6)
        spin.setValue(float(default))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(spin, 1)
        btn = QtWidgets.QPushButton("Set")
        btn.setFixedWidth(48)
        btn.clicked.connect(
            lambda _=False, f=field, c=cmd, s=spin: self._send_tuning(f, c, s)
        )
        row.addWidget(btn)
        wrap = QtWidgets.QWidget()
        wrap.setLayout(row)
        layout.addRow(label + ":", wrap)
        store[field] = spin

    def _send_coarse_centre(self) -> None:
        self._send_cmd(
            UPDATE_COARSE_SCAN_CENTRE_CMD,
            self.centre_x_spin.value(),
            self.centre_y_spin.value(),
        )

    def _send_p2p_target(self) -> None:
        self._send_cmd(
            SET_POINT2POINT_TARGET_CMD,
            self.p2p_x_spin.value(),
            self.p2p_y_spin.value(),
        )

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

    def _send_cmd(self, cmd: int, *values: float) -> None:
        if self.reader is None:
            self._append_log("Not connected")
            return
        self.reader.write_command(cmd, *values)

    def _send_tuning(
        self, field: str, cmd: int, spin: QtWidgets.QDoubleSpinBox
    ) -> None:
        value = spin.value()
        self._tuning_values[field] = value
        self._send_cmd(cmd, value)

    def _send_enable_toggle(self) -> None:
        self._send_cmd(DISABLE_ENABLE_CMD)

    def _send_debug_toggle(self) -> None:
        self._send_cmd(SET_DEBUG_MODE_CMD)

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

    def _style_debug_btn(self) -> None:
        if self._debug:
            self.debug_btn.setText("Debug on — click to disable")
            self.debug_btn.setStyleSheet(
                "background-color: #1f6b3a; color: white; font-weight: bold;"
            )
        else:
            self.debug_btn.setText("Debug off — click to enable")
            self.debug_btn.setStyleSheet(
                "background-color: #6b1f1f; color: white; font-weight: bold;"
            )

    def _on_control_thread(self, timestamp: float, packet: tuple) -> None:
        self.history.append(timestamp, packet)

    def _on_status_thread(self, packet: tuple) -> None:
        self.status_received.emit(packet)

    def _on_status_ui(self, packet: tuple) -> None:
        enabled = bool(packet[1])
        if enabled != self._enabled:
            self._enabled = enabled
            self._style_enable_btn()
        debug = bool(packet[4])
        if debug != self._debug:
            self._debug = debug
            self._style_debug_btn()


    def _toggle_pause(self, checked: bool) -> None:
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    def _update_plot(self) -> None:
        if self.paused:
            return
        tx, ty, mx, my = self.history.snapshot()
        if tx.size == 0:
            return
        self.target_curve.setData(tx, ty)
        self.mech_curve.setData(mx, my)

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
        super().closeEvent(event)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    window = RealtimePlotter()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
