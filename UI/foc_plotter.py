"""
FOC Realtime Plotter
Qt (PySide6 + pyqtgraph) tool that receives binary FOCStatusPacket telemetry
from the firmware (see `Interface::sendFOCStatus`) and plots selected channels
in real time for tuning and debugging. One or both motors can be overlaid on
the same axes (M1 solid, M2 dashed).

Packet layout (little-endian, packed, 66 bytes):
    uint8  header   (0xAA)
    uint8  motor    (1 or 2)
    float  angle, raw_velocity, filtered_velocity, target_angle,
           target_velocity, angular_error, velocity_error,
           target_iq, requested_vq, measured_iq,
           raw_adc_a, raw_adc_b, amps_a, amps_b, volts_a, volts_b

Any non-packet bytes (e.g. Serial.println text from the firmware) are decoded
and shown in the log panel at the bottom.

Usage:
    pip install PySide6 pyqtgraph pyserial
    python foc_plotter.py
"""

import struct
import sys
import threading
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtWidgets

# ---------- Protocol (mirrors src/interface/Interface.hpp) ----------
FOC_STATUS_HEADER = 0xAA  # 0b10101010
PACKET_STRUCT = struct.Struct("<BB16f")
PACKET_SIZE = PACKET_STRUCT.size  # 66 bytes
VALID_MOTORS = (1, 2)
# values[10] and values[11] are raw_adc_a / raw_adc_b after the 10 motion floats
RAW_ADC_A_INDEX = 10
RAW_ADC_B_INDEX = 11
MIN_PLAUSIBLE_RAW_ADC = 200
MAX_PLAUSIBLE_RAW_ADC = 4000

DEFAULT_BAUD = 460800

MOTOR_LABELS = {1: "M1", 2: "M2"}

# (name, unit, default pen colour, plotted by default)
CHANNELS = [
    ("Angle",             "rad",   "#ffd700", False),
    ("Raw Velocity",      "rad/s", "#666688", False),
    ("Filtered Velocity", "rad/s", "#00bfff", False),
    ("Target Angle",      "rad",   "#ff8c00", False),
    ("Target Velocity",   "rad/s", "#1e90ff", False),
    ("Angular Error",     "rad",   "#ff4444", False),
    ("Velocity Error",    "rad/s", "#ff44ff", False),
    ("Target Iq",         "A",     "#44ff44", False),
    ("Requested Vq",      "V",     "#44ffff", False),
    ("Measured Iq",       "A",     "#aaff00", False),
    ("Raw ADC A",         "LSB",   "#c080ff", True),
    ("Raw ADC B",         "LSB",   "#8080ff", True),
    ("Amps A",            "A",     "#ff8844", True),
    ("Amps B",            "A",     "#ff6688", True),
    ("Volts A",           "V",     "#88ffcc", True),
    ("Volts B",           "V",     "#66ccff", True),
]
NUM_CHANNELS = len(CHANNELS)

HISTORY_SAMPLES = 30000  # ~150 s at 200 Hz


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
            motor = buf[1]
            if motor not in VALID_MOTORS:
                # False header (e.g. 0xAA inside text); skip one byte
                self._consume_text(1)
                continue
            fields = PACKET_STRUCT.unpack_from(buf, 0)
            raw_a = fields[2 + RAW_ADC_A_INDEX]
            raw_b = fields[2 + RAW_ADC_B_INDEX]
            if not (
                MIN_PLAUSIBLE_RAW_ADC <= raw_a <= MAX_PLAUSIBLE_RAW_ADC
                and MIN_PLAUSIBLE_RAW_ADC <= raw_b <= MAX_PLAUSIBLE_RAW_ADC
            ):
                # Misaligned 0xAA — resync one byte at a time
                self._consume_text(1)
                continue
            del buf[:PACKET_SIZE]
            self.packet_sink(fields[1], time.monotonic(), fields[2:])
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
            for dq, v in zip(self.values, values):
                dq.append(v)
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
