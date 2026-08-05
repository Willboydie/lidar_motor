"""
Coarse room scanner — Qt applet for live LiDAR point mapping.

Talks to the Teensy ScanEngine bench (`src/_teensy_main.cpp`) over serial:
ControlPacket (0x55 @ ~1 kHz) and StatusPacket (0xAA @ 10 Hz), with framed
host→device commands matching `src/parser/Parser.hpp` / `cmd_defs.h`.

Maps (associated_angle_x, associated_angle_y, latest_distance) via Model.py
to world Cartesian (x, y, z). Plots X vs Y; Z is point colour.

Usage:
    pip install PySide6 pyqtgraph pyserial
    python -m pytools.coarse_scanning.CoarseRoomScanner
    # or:
    python CoarseRoomScanner.py
"""

from __future__ import annotations

import struct
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets

# Allow `python CoarseRoomScanner.py` from this directory
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))

from Model import (  # noqa: E402
    X_ANGLE_OFFSET,
    Y_ANGLE_OFFSET,
    computeCartPoint,
    mapToWorldAngles,
)

# ---------- Protocol (mirrors Interface.hpp / cmd_defs.h / Debug.hpp) ----------
CONTROL_HEADER = 0x55
STATUS_HEADER = 0xAA
DEBUG_HEADER = bytes((0xFF, 0x04, 0x06))
DEBUG_HEADER_SIZE = len(DEBUG_HEADER)
CONTROL_STRUCT = struct.Struct("<BffHffff")  # 27 B
STATUS_STRUCT = struct.Struct("<BBBBBfffffffffffff")  # 57 B
CONTROL_SIZE = CONTROL_STRUCT.size
STATUS_SIZE = STATUS_STRUCT.size

DISABLE_ENABLE_CMD = 0x01
ENABLE_SCANENGINE_CMD = 0x1D
DISABLE_SCANENGINE_CMD = 0x1E
BEGIN_COARSE_SCANNING_CMD = 0x1F
STOP_COARSE_SCANNING_CMD = 0x27
UPDATE_COARSE_SCAN_CENTRE_CMD = 0x2B

DEFAULT_BAUD = 460800
DEFAULT_CENTRE_RATE_RAD_S = 0.15
CENTRE_NUDGE_INTERVAL_MS = 20  # 50 Hz centre updates while arrow held
PLOT_INTERVAL_MS = 33  # ~30 Hz UI refresh
MAX_POINTS = 200_000

# ControlPacket field indices
_CONTROL_DISTANCE = 3
_CONTROL_ASSOC_X = 6
_CONTROL_ASSOC_Y = 7


class PointCloud:
    """Thread-safe store of Cartesian samples (only on distance change)."""

    def __init__(self, maxlen: int = MAX_POINTS):
        self.lock = threading.Lock()
        self.maxlen = maxlen
        self.xs: deque[float] = deque(maxlen=maxlen)
        self.ys: deque[float] = deque(maxlen=maxlen)
        self.zs: deque[float] = deque(maxlen=maxlen)
        self.distance_updates = 0
        self._last_distance: int | None = None

    def maybe_add(self, assoc_x: float, assoc_y: float, distance_cm: int) -> bool:
        """Add a point if distance changed. Returns True if a point was added."""
        with self.lock:
            if self._last_distance is not None and distance_cm == self._last_distance:
                return False
            self._last_distance = distance_cm
            r_m = distance_cm / 100.0
            x, y, z = computeCartPoint(assoc_x, assoc_y, r_m)
            self.xs.append(x)
            self.ys.append(y)
            self.zs.append(z)
            self.distance_updates += 1
            return True

    def clear(self) -> None:
        with self.lock:
            self.xs.clear()
            self.ys.clear()
            self.zs.clear()
            self._last_distance = None

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self.lock:
            if not self.xs:
                empty = np.empty(0, dtype=np.float64)
                return empty, empty, empty
            return (
                np.asarray(self.xs, dtype=np.float64),
                np.asarray(self.ys, dtype=np.float64),
                np.asarray(self.zs, dtype=np.float64),
            )


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


def _z_to_brushes(zs: np.ndarray) -> np.ndarray:
    """Map Z to a blue→cyan→yellow→red temperature scale (RGBA uint8 Nx4)."""
    if zs.size == 0:
        return np.empty((0, 4), dtype=np.uint8)
    z_min = float(np.min(zs))
    z_max = float(np.max(zs))
    span = max(z_max - z_min, 1e-9)
    t = (zs.astype(np.float64) - z_min) / span
    r = np.zeros_like(t)
    g = np.zeros_like(t)
    b = np.zeros_like(t)

    m0 = t < 0.25
    u = t[m0] / 0.25
    g[m0] = 80 + 175 * u
    b[m0] = 255

    m1 = (t >= 0.25) & (t < 0.5)
    u = (t[m1] - 0.25) / 0.25
    g[m1] = 255
    b[m1] = 255 * (1.0 - u)

    m2 = (t >= 0.5) & (t < 0.75)
    u = (t[m2] - 0.5) / 0.25
    r[m2] = 255 * u
    g[m2] = 255

    m3 = t >= 0.75
    u = (t[m3] - 0.75) / 0.25
    r[m3] = 255
    g[m3] = 255 * (1.0 - u)

    rgba = np.column_stack(
        [
            np.clip(r, 0, 255).astype(np.uint8),
            np.clip(g, 0, 255).astype(np.uint8),
            np.clip(b, 0, 255).astype(np.uint8),
            np.full(zs.shape, 200, dtype=np.uint8),
        ]
    )
    return rgba


class CoarseRoomScanner(QtWidgets.QMainWindow):
    text_received = QtCore.Signal(str)
    debug_received = QtCore.Signal(str)
    error_received = QtCore.Signal(str)
    status_received = QtCore.Signal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Coarse Room Scanner")
        self.resize(1280, 820)

        self.reader: SerialReader | None = None
        self.cloud = PointCloud()
        self._enabled = False
        self._centre_x = float(X_ANGLE_OFFSET)
        self._centre_y = float(Y_ANGLE_OFFSET)
        self._nudge_last_t = time.monotonic()
        self._rate_last_time = time.monotonic()
        self._rate_last_count = 0
        self._last_plotted_count = -1

        self.text_received.connect(self._append_log)
        self.debug_received.connect(self._append_debug)
        self.error_received.connect(self._on_serial_error)
        self.status_received.connect(self._on_status_ui)

        self._build_ui()
        self._update_centre_label()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(PLOT_INTERVAL_MS)

        self.rate_timer = QtCore.QTimer(self)
        self.rate_timer.timeout.connect(self._update_rate)
        self.rate_timer.start(1000)

        self.nudge_timer = QtCore.QTimer(self)
        self.nudge_timer.timeout.connect(self._nudge_centre)
        self.nudge_timer.start(CENTRE_NUDGE_INTERVAL_MS)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMaximumWidth(420)
        panel_widget = QtWidgets.QWidget()
        panel = QtWidgets.QVBoxLayout(panel_widget)
        panel.setSpacing(8)
        scroll.setWidget(panel_widget)
        root.addWidget(scroll, 0)

        # --- Connection ---
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

        # --- System ---
        state = QtWidgets.QGroupBox("System")
        state_layout = QtWidgets.QVBoxLayout(state)
        self.enable_btn = QtWidgets.QPushButton("Enable")
        self.enable_btn.clicked.connect(self._on_enable_clicked)
        self._style_enable_btn()
        state_layout.addWidget(self.enable_btn)
        panel.addWidget(state)

        # --- Scan ---
        scan_box = QtWidgets.QGroupBox("Coarse scan")
        scan_layout = QtWidgets.QVBoxLayout(scan_box)
        scan_row = QtWidgets.QHBoxLayout()
        self.scan_btn = QtWidgets.QPushButton("Scan")
        self.scan_btn.clicked.connect(self._on_scan)
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.clicked.connect(self._on_stop)
        scan_row.addWidget(self.scan_btn)
        scan_row.addWidget(self.stop_btn)
        scan_layout.addLayout(scan_row)

        rate_row = QtWidgets.QHBoxLayout()
        rate_row.addWidget(QtWidgets.QLabel("Centre rate R [rad/s]:"))
        self.rate_spin = QtWidgets.QDoubleSpinBox()
        self.rate_spin.setDecimals(3)
        self.rate_spin.setRange(0.001, 5.0)
        self.rate_spin.setSingleStep(0.05)
        self.rate_spin.setValue(DEFAULT_CENTRE_RATE_RAD_S)
        rate_row.addWidget(self.rate_spin)
        scan_layout.addLayout(rate_row)

        # Arrow pad: sticky (checkable) buttons
        pad = QtWidgets.QGridLayout()
        self.btn_up = QtWidgets.QPushButton("↑")
        self.btn_down = QtWidgets.QPushButton("↓")
        self.btn_left = QtWidgets.QPushButton("←")
        self.btn_right = QtWidgets.QPushButton("→")
        for b in (self.btn_up, self.btn_down, self.btn_left, self.btn_right):
            b.setCheckable(True)
            b.setFixedSize(56, 40)
            b.setStyleSheet(
                "QPushButton:checked { background-color: #2a5a8a; color: white; }"
            )
        pad.addWidget(self.btn_up, 0, 1)
        pad.addWidget(self.btn_left, 1, 0)
        pad.addWidget(self.btn_right, 1, 2)
        pad.addWidget(self.btn_down, 2, 1)
        pad_hint = QtWidgets.QLabel(
            "← +x   → −x\n↑ +y   ↓ −y\n(sticky while checked)"
        )
        pad_hint.setStyleSheet("color: #888888; font-size: 11px;")
        scan_layout.addLayout(pad)
        scan_layout.addWidget(pad_hint)

        self.centre_label = QtWidgets.QLabel()
        self.centre_label.setStyleSheet("font-family: Consolas, monospace;")
        scan_layout.addWidget(self.centre_label)
        panel.addWidget(scan_box)

        # --- Display ---
        disp = QtWidgets.QGroupBox("Display")
        disp_layout = QtWidgets.QVBoxLayout(disp)
        clear_btn = QtWidgets.QPushButton("Clear points")
        clear_btn.clicked.connect(self._clear_points)
        disp_layout.addWidget(clear_btn)
        self.dist_rate_label = QtWidgets.QLabel("Distance updates: 0 Hz")
        self.point_count_label = QtWidgets.QLabel("Points: 0")
        disp_layout.addWidget(self.dist_rate_label)
        disp_layout.addWidget(self.point_count_label)
        panel.addWidget(disp)

        panel.addStretch(1)

        # --- Plot + log ---
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        pg.setConfigOptions(antialias=False)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#101014")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "X", units="m")
        self.plot_widget.setLabel("left", "Y", units="m")
        self.plot_widget.setAspectLocked(True)
        self.scatter = pg.ScatterPlotItem(size=5, pen=None)
        self.plot_widget.addItem(self.scatter)
        self.z_range_label = QtWidgets.QLabel("Z colour: (no data)  blue→cyan→yellow→red")
        self.z_range_label.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        right.addWidget(self.plot_widget, 1)
        right.addWidget(self.z_range_label, 0)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(400)
        self.log_box.setFixedHeight(140)
        self.log_box.setPlaceholderText("Serial text / Debug::msg…")
        self.log_box.setStyleSheet(
            "background-color: #101014; color: #cccccc; font-family: Consolas, monospace;"
        )
        right.addWidget(self.log_box, 0)

        self._refresh_ports()

    # ---------- Centre tracking ----------
    def _reset_centre_to_neutral(self) -> None:
        self._centre_x = float(X_ANGLE_OFFSET)
        self._centre_y = float(Y_ANGLE_OFFSET)
        self._update_centre_label()

    def _update_centre_label(self) -> None:
        wx, wy = mapToWorldAngles(self._centre_x, self._centre_y)
        self.centre_label.setText(
            f"Scan centre (world):  x={wx:+.4f} rad   y={wy:+.4f} rad\n"
            f"Scan centre (motor): x={self._centre_x:.4f}   y={self._centre_y:.4f}"
        )

    def _send_centre(self) -> None:
        if self.reader is None:
            return
        self.reader.write_command(
            UPDATE_COARSE_SCAN_CENTRE_CMD, self._centre_x, self._centre_y
        )
        self._update_centre_label()

    def _nudge_centre(self) -> None:
        now = time.monotonic()
        dt = now - self._nudge_last_t
        self._nudge_last_t = now
        if self.reader is None or dt <= 0 or dt > 0.5:
            return

        dx = 0.0
        dy = 0.0
        r = self.rate_spin.value()
        # ← increases x, → decreases x; ↑ increases y, ↓ decreases y
        if self.btn_left.isChecked():
            dx += r * dt
        if self.btn_right.isChecked():
            dx -= r * dt
        if self.btn_up.isChecked():
            dy += r * dt
        if self.btn_down.isChecked():
            dy -= r * dt

        if dx == 0.0 and dy == 0.0:
            return

        self._centre_x += dx
        self._centre_y += dy
        self._send_centre()

    # ---------- Commands ----------
    def _clear_points(self) -> None:
        self.cloud.clear()
        self._last_plotted_count = -1

    def _on_enable_clicked(self) -> None:
        if self._enabled:
            self._send_cmd(DISABLE_ENABLE_CMD)
            self._send_cmd(DISABLE_SCANENGINE_CMD)
        else:
            self._send_cmd(DISABLE_ENABLE_CMD)
            self._send_cmd(ENABLE_SCANENGINE_CMD)

    def _on_scan(self) -> None:
        self._send_cmd(BEGIN_COARSE_SCANNING_CMD)

    def _on_stop(self) -> None:
        self._send_cmd(STOP_COARSE_SCANNING_CMD)
        # Firmware returns to neutral after taper; centre resets to original.
        for b in (self.btn_up, self.btn_down, self.btn_left, self.btn_right):
            b.setChecked(False)
        self._reset_centre_to_neutral()

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

    # ---------- Serial ----------
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
        self._reset_centre_to_neutral()

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

    def _on_control_thread(self, _timestamp: float, packet: tuple) -> None:
        self.cloud.maybe_add(
            float(packet[_CONTROL_ASSOC_X]),
            float(packet[_CONTROL_ASSOC_Y]),
            int(packet[_CONTROL_DISTANCE]),
        )

    def _on_status_thread(self, packet: tuple) -> None:
        self.status_received.emit(packet)

    def _on_status_ui(self, packet: tuple) -> None:
        enabled = bool(packet[1])
        if enabled != self._enabled:
            self._enabled = enabled
            self._style_enable_btn()

    # ---------- Plot / rates ----------
    def _update_plot(self) -> None:
        xs, ys, zs = self.cloud.snapshot()
        self.point_count_label.setText(f"Points: {xs.size}")
        if xs.size == self._last_plotted_count:
            return
        self._last_plotted_count = xs.size
        if xs.size == 0:
            self.scatter.setData([], [])
            self.z_range_label.setText("Z colour: (no data)  blue→cyan→yellow→red")
            return
        rgba = _z_to_brushes(zs)
        brushes = [pg.mkBrush(*tuple(c)) for c in rgba]
        self.scatter.setData(x=xs, y=ys, brush=brushes)
        z_min, z_max = float(np.min(zs)), float(np.max(zs))
        self.z_range_label.setText(
            f"Z colour [m]: {z_min:.3f} (blue) → {z_max:.3f} (red)"
        )

    def _update_rate(self) -> None:
        now = time.monotonic()
        dt = now - self._rate_last_time
        count = self.cloud.distance_updates
        rate = max(0.0, (count - self._rate_last_count) / dt) if dt > 0 else 0.0
        self._rate_last_count = count
        self._rate_last_time = now
        self.dist_rate_label.setText(f"Distance updates: {rate:.1f} Hz")

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
    window = CoarseRoomScanner()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
