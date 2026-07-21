"""
FOC Realtime Plotter
Qt (PySide6 + pyqtgraph) tool that receives binary DualMotorStatusPacket
telemetry from the firmware (see `Interface::sendDualMotorStatus`) and plots
selected channels in real time for tuning and debugging. Both motors arrive in
one packet and can be overlaid on the same axes (M1 solid, M2 dashed).

Packet layout (little-endian, packed, 97 bytes):
    uint8  header   (0xAA)
    MotorStatus m1  (12 floats: ia, ib, ic, id, iq, raw_angle, elec_angle, cmd_elec_angle, angular_error, va, vb, vc)
    MotorStatus m2  (12 floats: same order)
All MotorStatus fields are normalised to [-1, 1] on the wire (currents by
MOTOR_CURRENT_LIMIT, angles by 2*pi, angular error by pi, voltages by supply V).
The plotter denormalises to physical units before display.

Any non-packet bytes (e.g. Serial.println text from the firmware) are decoded
and shown in the log panel at the bottom.

Usage:
    pip install PySide6 pyqtgraph pyserial
    python foc_plotter.py
"""

import json
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
from PySide6 import QtCore, QtWidgets

# ---------- Protocol (mirrors src/interface/Interface.hpp) ----------
FOC_STATUS_HEADER = 0xAA  # 0b10101010
# header byte + 2 x MotorStatus (12 floats each)
PACKET_STRUCT = struct.Struct("<B24f")
PACKET_SIZE = PACKET_STRUCT.size  # 97 bytes
VALID_MOTORS = (1, 2)
FLOATS_PER_MOTOR = 12
# Fields are normalised to [-1, 1]; allow a small margin for resync validation.
MAX_PLAUSIBLE_VALUE = 1.5

# Denominators used by makeNormalisedMotorStatus() — multiply packet values to
# recover physical units before plotting.
MOTOR_CURRENT_LIMIT = 0.8
VOLTAGE_POWER_SUPPLY = 12.0
_PI = np.pi
_TWO_PI = 2.0 * np.pi

# Channel index constants — order matches MotorStatus in Interface.hpp
CH_IA, CH_IB, CH_IC = 0, 1, 2
CH_ID, CH_IQ = 3, 4
CH_RAW_ANGLE, CH_ELEC_ANGLE, CH_CMD_ELEC_ANGLE, CH_ANGLE_ERROR = 5, 6, 7, 8
CH_VA, CH_VB, CH_VC = 9, 10, 11

CHANNEL_DENORM = (
    MOTOR_CURRENT_LIMIT,
    MOTOR_CURRENT_LIMIT,
    MOTOR_CURRENT_LIMIT,
    MOTOR_CURRENT_LIMIT,
    MOTOR_CURRENT_LIMIT,
    _TWO_PI,
    _TWO_PI,
    _TWO_PI,
    _PI,
    VOLTAGE_POWER_SUPPLY,
    VOLTAGE_POWER_SUPPLY,
    VOLTAGE_POWER_SUPPLY,
)

DEFAULT_BAUD = 460800

MOTOR_LABELS = {1: "M1", 2: "M2"}

# (name, unit, default pen colour, plotted by default) — order matches MotorStatus
CHANNELS = [
    ("Ia",          "A",   "#ff8844", True),
    ("Ib",          "A",   "#ff6688", True),
    ("Ic",          "A",   "#ffaa44", True),
    ("Id",          "A",   "#44ff44", False),
    ("Iq",          "A",   "#aaff00", False),
    ("Raw Angle",   "rad", "#ffd700", True),
    ("Elec Angle",  "rad", "#ffa500", True),
    ("Cmd Elec Angle", "rad", "#ff8800", True),
    ("Angle Error", "rad", "#ff66ff", True),
    ("Va",          "V",   "#44ffff", True),
    ("Vb",          "V",   "#88ffcc", True),
    ("Vc",          "V",   "#66ccff", True),
]
NUM_CHANNELS = len(CHANNELS)

HISTORY_SAMPLES = 30000  # ~150 s at 200 Hz

# #region agent log
GM2804_POLE_PAIRS = 7
DEBUG_LOG_PATH = Path(__file__).resolve().parent.parent / "debug-107e68.log"
DEBUG_SESSION_ID = "107e68"
RAW_ANGLE_CH = CH_RAW_ANGLE
ELEC_ANGLE_CH = CH_ELEC_ANGLE
ANGLE_ERROR_CH = CH_ANGLE_ERROR


def _unwrap_delta(a: float, b: float) -> float:
    """Shortest signed angular delta a - b in radians."""
    return float((a - b + np.pi) % (2.0 * np.pi) - np.pi)


class DebugAngleLogger:
    """NDJSON logger for open-loop angle debugging (session 107e68)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._prev_packet: dict[int, tuple[float, float, float]] = {}
        self._packet_count = 0

    def _write(self, payload: dict) -> None:
        payload.setdefault("sessionId", DEBUG_SESSION_ID)
        payload.setdefault("timestamp", int(time.time() * 1000))
        line = json.dumps(payload, separators=(",", ":"))
        with self._lock:
            with DEBUG_LOG_PATH.open("a", encoding="utf-8") as fh:
                fh.write(line + "\n")

    def log_packet(self, motor: int, timestamp: float, values: tuple) -> None:
        self._packet_count += 1
        if self._packet_count % 3 != 0:
            return

        mech = float(values[RAW_ANGLE_CH]) * _TWO_PI
        elec = float(values[ELEC_ANGLE_CH]) * _TWO_PI
        angle_err = float(values[ANGLE_ERROR_CH]) * _PI
        prev = self._prev_packet.get(motor)
        if prev is not None:
            prev_mech, prev_elec, prev_ts = prev
            dt = timestamp - prev_ts
            d_mech = _unwrap_delta(mech, prev_mech)
            d_elec = _unwrap_delta(elec, prev_elec)
            mech_rate = d_mech / dt if dt > 0 else 0.0
            elec_rate = d_elec / dt if dt > 0 else 0.0
            rate_ratio = (mech_rate / elec_rate) if abs(elec_rate) > 1e-6 else None
            expected_ratio = 1.0 / GM2804_POLE_PAIRS
            elec_from_mech = (mech * GM2804_POLE_PAIRS) % _TWO_PI
            align_err = _unwrap_delta(elec, elec_from_mech)
            near_wrap = (mech < 0.15) or (mech > _TWO_PI - 0.15) or (elec < 0.15) or (
                elec > _TWO_PI - 0.15
            )
            jump = abs(d_mech) > 0.04 and abs(d_elec) < 0.08
            quant_step = _TWO_PI / 16384.0
            quant_like = abs(abs(d_mech) - quant_step) < quant_step * 0.35

            self._write(
                {
                    "runId": "pre-fix",
                    "hypothesisId": "H1-H5",
                    "location": "foc_plotter.py:log_packet",
                    "message": "angle_packet",
                    "data": {
                        "motor": motor,
                        "mech_rad": round(mech, 5),
                        "elec_rad": round(elec, 5),
                        "angle_err_rad": round(angle_err, 5),
                        "d_mech": round(d_mech, 6),
                        "d_elec": round(d_elec, 6),
                        "mech_rate": round(mech_rate, 4),
                        "elec_rate": round(elec_rate, 4),
                        "rate_ratio": round(rate_ratio, 4) if rate_ratio is not None else None,
                        "expected_rate_ratio": round(expected_ratio, 4),
                        "align_err_rad": round(align_err, 5),
                        "near_wrap": near_wrap,
                        "mech_jump": jump,
                        "quant_step_like": quant_like,
                        "dt_s": round(dt, 5),
                    },
                }
            )
        self._prev_packet[motor] = (mech, elec, timestamp)

    def log_firmware_line(self, line: str) -> None:
        if not line.startswith("DBG "):
            return
        parts = line[4:].split(",")
        if len(parts) < 6:
            return
        try:
            elec_cmd, mech, elec_est, align_err, d_mech, d_elec = map(float, parts[:6])
        except ValueError:
            return
        self._write(
            {
                "runId": "pre-fix",
                "hypothesisId": "H2",
                "location": "foc_plotter.py:log_firmware_line",
                "message": "firmware_alignment",
                "data": {
                    "elec_cmd": elec_cmd,
                    "mech_rad": mech,
                    "elec_from_sensor": elec_est,
                    "align_err_rad": align_err,
                    "d_mech_since_last_dbg": d_mech,
                    "d_elec_since_last_dbg": d_elec,
                },
            }
        )


_DEBUG_ANGLE_LOGGER = DebugAngleLogger()
# #endregion


def _motor_pen(base_colour: str, motor: int):
    """Solid pen for M1; dashed + slightly lighter for M2 so both are visible together."""
    colour = pg.mkColor(base_colour)
    if motor == 2:
        colour = colour.lighter(130)
    return pg.mkPen(
        colour,
        width=1.5 if motor == 1 else 1.25,
        style=QtCore.Qt.PenStyle.SolidLine if motor == 1 else QtCore.Qt.PenStyle.DashLine,
    )


class SerialReader(threading.Thread):
    """Reads the serial port, extracts FOCStatusPackets and text lines."""

    def __init__(self, port, baud, packet_sink, text_sink, error_sink):
        super().__init__(daemon=True)
        self.port_name = port
        self.baud = baud
        self.packet_sink = packet_sink  # callable(motor, timestamp, values)
        self.text_sink = text_sink      # callable(str)
        self.error_sink = error_sink    # callable(str)
        self._stop_event = threading.Event()
        self._serial = None
        self._rx_buffer = bytearray()
        self._text_buffer = bytearray()

    def stop(self):
        self._stop_event.set()

    def write(self, data: bytes):
        ser = self._serial
        if ser is not None and ser.is_open:
            try:
                ser.write(data)
            except serial.SerialException:
                pass

    def run(self):
        try:
            self._serial = serial.Serial(self.port_name, self.baud, timeout=0.05)
        except serial.SerialException as exc:
            self.error_sink(f"Failed to open {self.port_name}: {exc}")
            return

        try:
            while not self._stop_event.is_set():
                try:
                    chunk = self._serial.read(4096)
                except serial.SerialException as exc:
                    self.error_sink(f"Serial error: {exc}")
                    break
                if chunk:
                    self._rx_buffer.extend(chunk)
                    self._parse_buffer()
        finally:
            try:
                self._serial.close()
            except serial.SerialException:
                pass

    def _parse_buffer(self):
        buf = self._rx_buffer
        while True:
            idx = buf.find(FOC_STATUS_HEADER)
            if idx < 0:
                # No header in sight: everything is text
                self._consume_text(len(buf))
                break
            # Bytes before the header are text
            if idx > 0:
                self._consume_text(idx)
            if len(buf) < PACKET_SIZE:
                break  # wait for the rest of the packet
            fields = PACKET_STRUCT.unpack_from(buf, 0)
            values = fields[1:]  # 22 floats: m1 (11) then m2 (11)
            if not all(abs(v) <= MAX_PLAUSIBLE_VALUE for v in values):
                # Misaligned 0xAA (e.g. inside text/floats) — resync one byte at a time
                self._consume_text(1)
                continue
            del buf[:PACKET_SIZE]
            ts = time.monotonic()
            self.packet_sink(1, ts, values[:FLOATS_PER_MOTOR])
            self.packet_sink(2, ts, values[FLOATS_PER_MOTOR:])
        self._flush_text()

    def _consume_text(self, n):
        self._text_buffer.extend(self._rx_buffer[:n])
        del self._rx_buffer[:n]

    def _flush_text(self):
        # Emit complete lines from the text buffer
        while True:
            idx = self._text_buffer.find(b"\n")
            if idx < 0:
                break
            line = self._text_buffer[: idx + 1]
            del self._text_buffer[: idx + 1]
            text = line.decode("ascii", errors="replace").strip()
            if text:
                self.text_sink(text)
        # Avoid unbounded growth if firmware never sends a newline
        if len(self._text_buffer) > 4096:
            self._text_buffer.clear()


class MotorHistory:
    """Ring buffers of timestamps + channel values for one motor."""

    def __init__(self):
        self.lock = threading.Lock()
        self.times = deque(maxlen=HISTORY_SAMPLES)
        self.values = [deque(maxlen=HISTORY_SAMPLES) for _ in range(NUM_CHANNELS)]
        self.packet_count = 0

    def append(self, timestamp, values):
        with self.lock:
            self.times.append(timestamp)
            for dq, v, scale in zip(self.values, values, CHANNEL_DENORM):
                dq.append(v * scale)
            self.packet_count += 1

    def clear(self):
        with self.lock:
            self.times.clear()
            for dq in self.values:
                dq.clear()

    def snapshot(self, channel_indices):
        with self.lock:
            t = np.fromiter(self.times, dtype=np.float64, count=len(self.times))
            chans = {
                i: np.fromiter(self.values[i], dtype=np.float64, count=len(self.values[i]))
                for i in channel_indices
            }
        return t, chans


class FOCPlotter(QtWidgets.QMainWindow):
    text_received = QtCore.Signal(str)
    error_received = QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOC Realtime Plotter")
        self.resize(1280, 800)

        self.reader = None
        self.histories = {1: MotorHistory(), 2: MotorHistory()}
        self.paused = False
        self._rate_last_time = time.monotonic()
        self._rate_last_counts = {1: 0, 2: 0}

        self.text_received.connect(self._append_log)
        self.error_received.connect(self._on_serial_error)

        self._build_ui()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(33)  # ~30 fps

        self.rate_timer = QtCore.QTimer(self)
        self.rate_timer.timeout.connect(self._update_rate)
        self.rate_timer.start(1000)

    # ---------- UI ----------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # Left: control panel
        panel = QtWidgets.QVBoxLayout()
        panel.setSpacing(8)
        root.addLayout(panel, 0)

        # Connection group
        conn_group = QtWidgets.QGroupBox("Connection")
        conn_layout = QtWidgets.QGridLayout(conn_group)
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.baud_edit = QtWidgets.QLineEdit(str(DEFAULT_BAUD))
        self.baud_edit.setMaximumWidth(90)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #ff5555;")
        conn_layout.addWidget(QtWidgets.QLabel("Port:"), 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(self.refresh_btn, 0, 2)
        conn_layout.addWidget(QtWidgets.QLabel("Baud:"), 1, 0)
        conn_layout.addWidget(self.baud_edit, 1, 1)
        conn_layout.addWidget(self.connect_btn, 1, 2)
        conn_layout.addWidget(self.status_label, 2, 0, 1, 3)
        panel.addWidget(conn_group)

        # Motor overlay — check one or both to plot simultaneously
        motor_group = QtWidgets.QGroupBox("Motors")
        motor_layout = QtWidgets.QVBoxLayout(motor_group)
        self.motor_checks = {}
        motor_btn_row = QtWidgets.QHBoxLayout()
        for n in VALID_MOTORS:
            cb = QtWidgets.QCheckBox(f"Motor {n} ({MOTOR_LABELS[n]})")
            cb.setChecked(True)
            cb.toggled.connect(self._on_motor_toggled)
            self.motor_checks[n] = cb
            motor_layout.addWidget(cb)
        all_m_btn = QtWidgets.QPushButton("Both")
        one_m_btn = QtWidgets.QPushButton("M1 only")
        all_m_btn.clicked.connect(lambda: self._set_all_motors(True))
        one_m_btn.clicked.connect(lambda: self._set_all_motors(False))
        motor_btn_row.addWidget(all_m_btn)
        motor_btn_row.addWidget(one_m_btn)
        motor_layout.addLayout(motor_btn_row)
        panel.addWidget(motor_group)

        # Channel checkboxes
        chan_group = QtWidgets.QGroupBox("Channels")
        chan_layout = QtWidgets.QVBoxLayout(chan_group)
        self.channel_checks = []
        for i, (name, unit, colour, default_on) in enumerate(CHANNELS):
            cb = QtWidgets.QCheckBox(f"{name} [{unit}]")
            cb.setChecked(default_on)
            cb.setStyleSheet(f"color: {colour};")
            chan_layout.addWidget(cb)
            self.channel_checks.append(cb)
        btn_row = QtWidgets.QHBoxLayout()
        all_btn = QtWidgets.QPushButton("All")
        none_btn = QtWidgets.QPushButton("None")
        all_btn.clicked.connect(lambda: self._set_all_channels(True))
        none_btn.clicked.connect(lambda: self._set_all_channels(False))
        btn_row.addWidget(all_btn)
        btn_row.addWidget(none_btn)
        chan_layout.addLayout(btn_row)
        panel.addWidget(chan_group)

        # Display settings
        disp_group = QtWidgets.QGroupBox("Display")
        disp_layout = QtWidgets.QGridLayout(disp_group)
        disp_layout.addWidget(QtWidgets.QLabel("Window [s]:"), 0, 0)
        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(0.5, 120.0)
        self.window_spin.setValue(10.0)
        self.window_spin.setSingleStep(1.0)
        disp_layout.addWidget(self.window_spin, 0, 1)
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._toggle_pause)
        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.clicked.connect(self._clear_data)
        disp_layout.addWidget(self.pause_btn, 1, 0)
        disp_layout.addWidget(self.clear_btn, 1, 1)
        panel.addWidget(disp_group)

        self.rate_label = QtWidgets.QLabel("0 packets/s")
        panel.addWidget(self.rate_label)
        panel.addStretch(1)

        # Right: plot + log
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        pg.setConfigOptions(antialias=False)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#101014")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "time", units="s")
        self.legend = self.plot_widget.addLegend(offset=(10, 10))
        self.curves = {}
        for motor in VALID_MOTORS:
            for i, (name, unit, colour, _) in enumerate(CHANNELS):
                label = f"{MOTOR_LABELS[motor]} {name} [{unit}]"
                self.curves[(motor, i)] = self.plot_widget.plot(
                    [], [], pen=_motor_pen(colour, motor), name=label
                )
        right.addWidget(self.plot_widget, 1)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(500)
        self.log_box.setFixedHeight(140)
        self.log_box.setStyleSheet(
            "background-color: #101014; color: #cccccc; font-family: Consolas, monospace;"
        )
        right.addWidget(self.log_box, 0)

        self._refresh_ports()

    # ---------- Connection ----------
    def _refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _toggle_connection(self):
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
            packet_sink=self._on_packet,
            text_sink=self.text_received.emit,
            error_sink=self.error_received.emit,
        )
        self.reader.start()
        self.connect_btn.setText("Disconnect")
        self.status_label.setText(f"Connected: {port} @ {baud}")
        self.status_label.setStyleSheet("color: #55ff55;")
        self._append_log(f"Connected to {port} @ {baud}")

    def _disconnect(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.connect_btn.setText("Connect")
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: #ff5555;")
        self._append_log("Disconnected")

    def _on_serial_error(self, message):
        self._append_log(message)
        self._disconnect()

    # ---------- Data ----------
    def _on_packet(self, motor, timestamp, values):
        # Called from the reader thread
        # #region agent log
        _DEBUG_ANGLE_LOGGER.log_packet(motor, timestamp, values)
        # #endregion
        history = self.histories.get(motor)
        if history is not None:
            history.append(timestamp, values)

    def _clear_data(self):
        for history in self.histories.values():
            history.clear()

    def _set_all_channels(self, on):
        for cb in self.channel_checks:
            cb.setChecked(on)

    def _set_all_motors(self, on):
        for cb in self.motor_checks.values():
            cb.setChecked(on)
        if not on:
            self.motor_checks[1].setChecked(True)

    def _enabled_motors(self):
        return [m for m in VALID_MOTORS if self.motor_checks[m].isChecked()]

    def _on_motor_toggled(self):
        if not any(cb.isChecked() for cb in self.motor_checks.values()):
            sender = self.sender()
            if isinstance(sender, QtWidgets.QCheckBox):
                sender.setChecked(True)
            else:
                self.motor_checks[1].setChecked(True)
        self._update_plot()

    def _toggle_pause(self, checked):
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    # ---------- Plotting ----------
    def _update_plot(self):
        if self.paused:
            return
        enabled_motors = self._enabled_motors()
        enabled_channels = [i for i, cb in enumerate(self.channel_checks) if cb.isChecked()]
        now = time.monotonic()
        window = self.window_spin.value()

        for motor in VALID_MOTORS:
            show_motor = motor in enabled_motors
            t, chans = self.histories[motor].snapshot(enabled_channels)
            if t.size:
                rel_t = t - now
                start = np.searchsorted(rel_t, -window)
            else:
                rel_t = t
                start = 0

            for i in range(NUM_CHANNELS):
                curve = self.curves[(motor, i)]
                if show_motor and i in chans and t.size:
                    curve.setData(rel_t[start:], chans[i][start:])
                    curve.show()
                else:
                    curve.setData([], [])
                    if not show_motor:
                        curve.hide()

        self.plot_widget.setXRange(-window, 0, padding=0)

    def _update_rate(self):
        now = time.monotonic()
        dt = now - self._rate_last_time
        rates = {}
        for motor in VALID_MOTORS:
            count = self.histories[motor].packet_count
            prev = self._rate_last_counts[motor]
            rates[motor] = max(0.0, (count - prev) / dt) if dt > 0 else 0.0
            self._rate_last_counts[motor] = count
        self._rate_last_time = now
        self.rate_label.setText(
            f"M1: {rates[1]:.0f}  M2: {rates[2]:.0f} pkt/s"
        )

    # ---------- Log ----------
    def _append_log(self, text):
        # #region agent log
        _DEBUG_ANGLE_LOGGER.log_firmware_line(text)
        # #endregion
        self.log_box.appendPlainText(text)

    def closeEvent(self, event):
        self._disconnect()
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = FOCPlotter()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
