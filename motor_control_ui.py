"""
SimpleFOC Interface Control UI
Tkinter app that sends Interface command packets (1 byte command + 4 byte
little-endian float) so it matches the firmware `Interface` class.
"""

import struct
import threading
import time
import tkinter as tk
from queue import Queue
from tkinter import scrolledtext, ttk

import serial
import serial.tools.list_ports

# Command IDs mirrored from `src/interface/Interface.hpp`
DISABLE_ENABLE_CMD = 0b00000001
SCAN_PLAY_PAUSE_CMD = 0b00000010
TOGGLE_CONTROL_MODE_CMD = 0b00000011
VELOCITY_P_SET_CMD = 0b00001000
VELOCITY_I_SET_CMD = 0b00001001
VELOCITY_D_SET_CMD = 0b00001010
VELOCITY_RAMP_SET_CMD = 0b00001011
ANGLE_P_SET_CMD = 0b00001100
ANGLE_I_SET_CMD = 0b00001101
ANGLE_D_SET_CMD = 0b00001110
ANGLE_RAMP_SET_CMD = 0b00001111
SENSOR_VELOCITY_LPF_TAU_SET_CMD = 0b00010011
FOC_VOLTAGE_LPF_TAU_SET_CMD = 0b00010001
TARGET_SET_CMD = 0b00010000


class MotorControlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Interface Command UI")
        self.root.geometry("900x650")

        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        self.message_queue = Queue()

        self.setup_ui()
        self.update_serial_monitor()

    # ---------- UI SETUP ----------
    def setup_ui(self):
        """Create the user interface."""
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Connection
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, padx=5)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Label(conn_frame, textvariable=self.baud_var).grid(row=0, column=4, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)

        ttk.Button(conn_frame, text="Clear Monitor", command=self.clear_monitor).grid(
            row=0, column=7, padx=5, sticky="e"
        )
        conn_frame.grid_columnconfigure(7, weight=1)

        # Controls
        control_frame = ttk.LabelFrame(self.root, text="Interface Commands", padding=10)
        control_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=1)

        state_frame = ttk.LabelFrame(control_frame, text="State", padding=10)
        state_frame.grid(row=0, column=0, padx=5, pady=5, sticky="new")
        ttk.Button(state_frame, text="Enable / Disable", command=self.handle_enable_toggle).grid(
            row=0, column=0, padx=5, pady=5
        )
        ttk.Button(state_frame, text="Scan Play / Pause", command=self.handle_scan_toggle).grid(
            row=0, column=1, padx=5, pady=5
        )
        ttk.Button(state_frame, text="Toggle Control Mode", command=self.handle_mode_toggle).grid(
            row=0, column=2, padx=5, pady=5
        )

        velocity_frame = ttk.LabelFrame(control_frame, text="Velocity PID", padding=10)
        velocity_frame.grid(row=1, column=0, padx=5, pady=2, sticky="new")
        self.vel_p_entry = self._add_pid_row(velocity_frame, "P", 0, VELOCITY_P_SET_CMD)
        self.vel_i_entry = self._add_pid_row(velocity_frame, "I", 1, VELOCITY_I_SET_CMD)
        self.vel_d_entry = self._add_pid_row(velocity_frame, "D", 2, VELOCITY_D_SET_CMD)
        self.vel_ramp_entry = self._add_pid_row(velocity_frame, "Ramp", 3, VELOCITY_RAMP_SET_CMD)

        lpf_frame = ttk.LabelFrame(control_frame, text="Filters / LPF", padding=10)
        lpf_frame.grid(row=0, column=1, padx=5, pady=(0, 2), sticky="ew")
        self.sensor_vel_lpf_entry = self._add_pid_row(
            lpf_frame, "Sensor vel τ", 0, SENSOR_VELOCITY_LPF_TAU_SET_CMD
        )
        self.foc_voltage_lpf_entry = self._add_pid_row(
            lpf_frame, "FOC voltage τ", 1, FOC_VOLTAGE_LPF_TAU_SET_CMD
        )

        angle_frame = ttk.LabelFrame(control_frame, text="Angle PID", padding=10)
        angle_frame.grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        self.ang_p_entry = self._add_pid_row(angle_frame, "P", 0, ANGLE_P_SET_CMD)
        self.ang_i_entry = self._add_pid_row(angle_frame, "I", 1, ANGLE_I_SET_CMD)
        self.ang_d_entry = self._add_pid_row(angle_frame, "D", 2, ANGLE_D_SET_CMD)
        self.ang_ramp_entry = self._add_pid_row(angle_frame, "Ramp", 3, ANGLE_RAMP_SET_CMD)

        target_frame = ttk.LabelFrame(
            control_frame,
            text="Target (angle or velocity depends on firmware mode)",
            padding=10,
        )
        target_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="new")
        ttk.Label(target_frame, text="Target value:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.target_entry = ttk.Entry(target_frame, width=15)
        self.target_entry.insert(0, "0.0")
        self.target_entry.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(target_frame, text="Send Target", command=self.send_target).grid(row=0, column=2, padx=5, pady=5)

        # Serial Monitor
        monitor_frame = ttk.Frame(control_frame)
        monitor_frame.grid(row=3, column=0, columnspan=2, pady=10, sticky="nsew")
        monitor_frame.grid_rowconfigure(1, weight=1)
        monitor_frame.grid_columnconfigure(0, weight=1)

        ttk.Label(monitor_frame, text="Serial Monitor", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w")
        self.serial_text = scrolledtext.ScrolledText(
            monitor_frame, height=15, width=80, bg="black", fg="lime", font=("Courier", 9)
        )
        self.serial_text.grid(row=1, column=0, sticky="nsew")
        monitor_frame.grid_columnconfigure(0, weight=1)

    def _add_pid_row(self, frame, label, row, command):
        ttk.Label(frame, text=f"{label}:").grid(row=row, column=0, padx=5, pady=3, sticky="e")
        entry = ttk.Entry(frame, width=10)
        entry.insert(0, "0.0")
        entry.grid(row=row, column=1, padx=5, pady=3)
        ttk.Button(frame, text="Set", command=lambda e=entry, cmd=command: self.send_pid_value(e, cmd)).grid(
            row=row, column=2, padx=5, pady=3
        )
        return entry

    # ---------- SERIAL HANDLING ----------
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo["values"] = port_list
        if port_list:
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        port = self.port_combo.get()
        if not port:
            self.log_message("Error: No port selected\n")
            return

        try:
            baud = int(self.baud_var.get())
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.serial_port.dtr = True  # keep Pico USB serial happy
            self.is_connected = True
            self.running = True

            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

            self.status_label.config(text=f"Connected to {port}", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.log_message(f"=== Connected to {port} at {baud} baud ===\n")
        except Exception as e:
            self.log_message(f"Error connecting: {str(e)}\n")
            self.is_connected = False

    def disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)

        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None

        self.is_connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.log_message("=== Disconnected ===\n")

    def read_serial(self):
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode("utf-8", errors="ignore")
                    self.message_queue.put(data)
            except Exception as e:
                if self.running:
                    self.message_queue.put(f"Read error: {str(e)}\n")
                break
            time.sleep(0.01)

    # ---------- COMMANDS ----------
    def send_packet(self, command, value=0.0):
        if not self.is_connected or not self.serial_port:
            self.log_message("Error: Not connected to serial port\n")
            return

        try:
            packet = struct.pack("<Bf", command, float(value))
            self.serial_port.write(packet)
            # self.log_message(f">>> Sent cmd=0x{command:02X} data={float(value):.4f}\n")
        except Exception as e:
            self.log_message(f"Error sending command: {str(e)}\n")

    def handle_enable_toggle(self):
        self.send_packet(DISABLE_ENABLE_CMD, 0.0)

    def handle_scan_toggle(self):
        self.send_packet(SCAN_PLAY_PAUSE_CMD, 0.0)

    def handle_mode_toggle(self):
        self.send_packet(TOGGLE_CONTROL_MODE_CMD, 0.0)

    def send_target(self):
        value = self._read_float(self.target_entry, "target")
        if value is not None:
            self.send_packet(TARGET_SET_CMD, value)

    def send_pid_value(self, entry, command):
        value = self._read_float(entry, "PID value")
        if value is not None:
            self.send_packet(command, value)

    # ---------- UI UTILITIES ----------
    def _read_float(self, entry, label):
        try:
            return float(entry.get())
        except ValueError:
            self.log_message(f"Error: Invalid {label}\n")
            return None

    def update_serial_monitor(self):
        while not self.message_queue.empty():
            message = self.message_queue.get()
            self.serial_text.insert(tk.END, message)
            self.serial_text.see(tk.END)
        self.root.after(50, self.update_serial_monitor)

    def log_message(self, message):
        self.serial_text.insert(tk.END, message)
        self.serial_text.see(tk.END)

    def clear_monitor(self):
        self.serial_text.delete(1.0, tk.END)


def main():
    root = tk.Tk()
    app = MotorControlUI(root)

    def on_closing():
        if app.is_connected:
            app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()

