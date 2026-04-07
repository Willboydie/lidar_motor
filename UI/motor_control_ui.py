"""
SimpleFOC Interface Control UI
Tkinter app that sends Interface command packets (1 byte command + 4 byte
little-endian float) so it matches the firmware `Interface` class.
Also receives and displays DataPacket sensor readings.
"""

import struct
import threading
import time
import tkinter as tk
from collections import deque
from queue import Queue
from tkinter import scrolledtext, ttk
import os

import serial as serial
import serial.tools.list_ports

# Incoming data packet identifier (from DataStream.cpp)
SEND_DATA_CMD = 0x55  # 0b01010101
SEND_DATA_CMD_BYTES = bytes([SEND_DATA_CMD])  # Pre-allocated for efficient searching
DATA_PACKET_SIZE = 19  # 1 (cmd) + 2 (distance) + 4 (angle1) + 4 (angle2) + 4 (target1) + 4 (target2)
DATA_PACKET_FORMAT = "<BHffff"  # little-endian: uint8, uint16, float, float, float, float
DATA_PACKET_STRUCT = struct.Struct(DATA_PACKET_FORMAT)  # Pre-compiled for faster unpacking

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
VELOCITY_LPF_TAU_SET_CMD = 0b00010010
ANGLE_LPF_TAU_SET_CMD = 0b00010011
FOC_VOLTAGE_LPF_TAU_SET_CMD = 0b00010001
VELOCITY_FF_SET_CMD = 0b10010010
RECONFIGURE_LDM_CMD = 0b00100000
THETA_PHASE_INC_SET_CMD = 0b00100001
PHI_PHASE_INC_SET_CMD = 0b00100010
THETA_AMPLITUDE_SET_CMD = 0b00100011
PHI_AMPLITUDE_SET_CMD = 0b00100100
THETA_CENTRE_SET_CMD = 0b00100101
PHI_CENTRE_SET_CMD = 0b00100110
TARGET_1_SET_CMD = 0b00010000
TARGET_2_SET_CMD = 0b00010100

# Default values mirrored from `src/utils/tweakable_params.h` and scan path defaults
DEFAULTS = {
    "velocity_p": 0.0,
    "velocity_i": 0.0,
    "velocity_d": 0.0,
    "velocity_ramp": 0.0,
    "angle_p": 0.0,
    "angle_i": 0.0,
    "angle_d": 0.0,
    "angle_ramp": 0.0,
    "foc_voltage_lpf_tau": 0.001,
    "kalman_qa": 1e-9,
    "lpf_velocity_tau": 0.001,
    "velocity_ff": 0.0,
    "notch_filter_freq": 50.0,
    "notch_filter_q": 1.0,
    "notch_filter_sample_rate": 10000.0,
    "target_1": 0.0,
    "target_2": 0.0,
    "theta_phase_inc": 1500007,
    "phi_phase_inc": 3200003,
    "theta_amplitude": 0.79,
    "phi_amplitude": 0.3,
    "theta_centre": 0.0,
    "phi_centre": -0.3,
    "ldm_config": 0,
}


class MotorControlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Interface Command UI")
        self.root.geometry("1000x750")

        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        self.message_queue = Queue()
        self.data_queue = Queue()
        
        # Ring buffer for recent data points (for potential plotting)
        self.data_history = deque(maxlen=1000)
        
        # Recording state
        self.is_recording = False
        self.record_file = None
        self.recorded_packets = 0
        self.record_buffer = bytearray()  # Buffer for batching file writes
        self.record_buffer_size = 0
        self.record_flush_threshold = 4096  # Flush to disk every 4KB (~215 packets)

        self.setup_ui()
        self.update_serial_monitor()
        self.update_sensor_display()

    # ---------- UI SETUP ----------
    def setup_ui(self):
        """Create the user interface with tabbed pages."""
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Connection bar (always visible at top)
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, padx=5)
        self.baud_var = tk.StringVar(value="460800")
        ttk.Label(conn_frame, textvariable=self.baud_var).grid(row=0, column=4, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)

        conn_frame.grid_columnconfigure(7, weight=1)

        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        self.root.grid_rowconfigure(1, weight=1)

        # Create tabs
        self.settings_tab = ttk.Frame(self.notebook)
        self.data_tab = ttk.Frame(self.notebook)
        
        self.notebook.add(self.settings_tab, text="Settings Dashboard")
        self.notebook.add(self.data_tab, text="Incoming Data Dashboard")

        # Setup content for each tab
        self.setup_settings_tab()
        self.setup_data_tab()

    def setup_settings_tab(self):
        """Setup the Settings Dashboard tab."""
        self.settings_tab.grid_rowconfigure(0, weight=1)
        self.settings_tab.grid_columnconfigure(0, weight=1)

        # Main content frame
        main_frame = ttk.Frame(self.settings_tab)
        main_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(1, weight=1)

        # Top section: State Controls (full width)
        state_section = ttk.LabelFrame(main_frame, text="System State", padding=5)
        state_section.grid(row=0, column=0, sticky="ew", pady=(10, 10), columnspan=2)
        state_section.grid_columnconfigure(0, weight=1)
        
        state_buttons_frame = ttk.Frame(state_section)
        state_buttons_frame.grid(row=0, column=0, pady=5)
        
        ttk.Button(
            state_buttons_frame, 
            text="Enable / Disable", 
            command=self.handle_enable_toggle,
            width=18
        ).grid(row=0, column=0, padx=8, pady=5)
        ttk.Button(
            state_buttons_frame, 
            text="Toggle Control Mode", 
            command=self.handle_mode_toggle,
            width=18
        ).grid(row=0, column=1, padx=8, pady=5)
        ttk.Button(
            state_buttons_frame, 
            text="Scan Play / Pause", 
            command=self.handle_scan_toggle,
            width=18
        ).grid(row=0, column=2, padx=8, pady=5)

        # Middle section: Control Parameters (two columns)
        controls_container = ttk.Frame(main_frame)
        controls_container.grid(row=1, column=0, sticky="nsew", pady=(0, 10))
        controls_container.grid_columnconfigure(0, weight=1)
        controls_container.grid_columnconfigure(1, weight=1)
        controls_container.grid_rowconfigure(0, weight=1)

        # Left column: PID Controllers
        left_column = ttk.Frame(controls_container)
        left_column.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        # Velocity PID
        velocity_frame = ttk.LabelFrame(left_column, text="Velocity PID Controller", padding=12)
        velocity_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        self.vel_p_entry = self._add_pid_row(velocity_frame, "Proportional (P)", 0, VELOCITY_P_SET_CMD, "velocity_p")
        self.vel_i_entry = self._add_pid_row(velocity_frame, "Integral (I)", 1, VELOCITY_I_SET_CMD, "velocity_i")
        self.vel_d_entry = self._add_pid_row(velocity_frame, "Derivative (D)", 2, VELOCITY_D_SET_CMD, "velocity_d")
        self.vel_ramp_entry = self._add_pid_row(velocity_frame, "Ramp Rate", 3, VELOCITY_RAMP_SET_CMD, "velocity_ramp")

        # Angle PID
        angle_frame = ttk.LabelFrame(left_column, text="Angle PID Controller", padding=12)
        angle_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        self.ang_p_entry = self._add_pid_row(angle_frame, "Proportional (P)", 0, ANGLE_P_SET_CMD, "angle_p")
        self.ang_i_entry = self._add_pid_row(angle_frame, "Integral (I)", 1, ANGLE_I_SET_CMD, "angle_i")
        self.ang_d_entry = self._add_pid_row(angle_frame, "Derivative (D)", 2, ANGLE_D_SET_CMD, "angle_d")
        self.ang_ramp_entry = self._add_pid_row(angle_frame, "Ramp Rate", 3, ANGLE_RAMP_SET_CMD, "angle_ramp")

        # Filters & Feed-Forward
        lpf_frame = ttk.LabelFrame(left_column, text="Filters & Feed-Forward", padding=12)
        lpf_frame.grid(row=2, column=0, sticky="ew")
        self.lpf_velocity_tau_entry = self._add_pid_row(
            lpf_frame, "Sensor Velocity τ", 0, VELOCITY_LPF_TAU_SET_CMD, "lpf_velocity_tau"
        )
        self.angle_lpf_tau_entry = self._add_pid_row(
            lpf_frame, "Angle τ", 1, ANGLE_LPF_TAU_SET_CMD, "angle_lpf_tau"
        )
        self.foc_voltage_lpf_entry = self._add_pid_row(
            lpf_frame, "FOC Voltage τ", 2, FOC_VOLTAGE_LPF_TAU_SET_CMD, "foc_voltage_lpf_tau"
        )

        # self.kalman_qa_entry = self._add_pid_row(
        #     lpf_frame, "Kalman Qa", 1, KALMAN_QA_SET_CMD, "kalman_qa"
        # )
        self.velocity_ff_entry = self._add_pid_row(
            lpf_frame, "Velocity FF", 3, VELOCITY_FF_SET_CMD, "velocity_ff"
        )
        # self.notch_filter_freq_entry = self._add_pid_row(
        #     lpf_frame, "Notch Frequency", 3, NOTCH_FILTER_FREQ_SET_CMD, "notch_filter_freq"
        # )
        # self.notch_filter_q_entry = self._add_pid_row(
        #     lpf_frame, "Notch Q", 4, NOTCH_FILTER_Q_SET_CMD, "notch_filter_q"
        # )
        # self.notch_filter_sample_rate_entry = self._add_pid_row(
        #     lpf_frame, "Notch Sample Rate", 5, NOTCH_FILTER_SAMPLE_RATE_SET_CMD, "notch_filter_sample_rate"
        # )
        # Right column: Scan Path & System Config
        right_column = ttk.Frame(controls_container)
        right_column.grid(row=0, column=1, sticky="nsew", padx=(5, 0))

        # Scan Path Parameters
        scan_path_frame = ttk.LabelFrame(right_column, text="Scan Path Configuration", padding=12)
        scan_path_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        
        # Theta section
        theta_subframe = ttk.Frame(scan_path_frame)
        theta_subframe.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        ttk.Label(
            theta_subframe, 
            text="Theta Axis", 
            font=("Arial", 10, "bold")
        ).grid(row=0, column=0, sticky="w", padx=(0, 10))
        ttk.Separator(theta_subframe, orient="horizontal").grid(row=0, column=1, sticky="ew", padx=5)
        theta_subframe.grid_columnconfigure(1, weight=1)
        
        self.theta_phase_inc_entry = self._add_pid_row(scan_path_frame, "Phase Increment", 1, THETA_PHASE_INC_SET_CMD, "theta_phase_inc")
        self.theta_amplitude_entry = self._add_pid_row(scan_path_frame, "Amplitude", 2, THETA_AMPLITUDE_SET_CMD, "theta_amplitude")
        self.theta_centre_entry = self._add_pid_row(scan_path_frame, "Centre Offset", 3, THETA_CENTRE_SET_CMD, "theta_centre")
        
        # Phi section
        phi_subframe = ttk.Frame(scan_path_frame)
        phi_subframe.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(10, 8))
        ttk.Label(
            phi_subframe, 
            text="Phi Axis", 
            font=("Arial", 10, "bold")
        ).grid(row=0, column=0, sticky="w", padx=(0, 10))
        ttk.Separator(phi_subframe, orient="horizontal").grid(row=0, column=1, sticky="ew", padx=5)
        phi_subframe.grid_columnconfigure(1, weight=1)
        
        self.phi_phase_inc_entry = self._add_pid_row(scan_path_frame, "Phase Increment", 5, PHI_PHASE_INC_SET_CMD, "phi_phase_inc")
        self.phi_amplitude_entry = self._add_pid_row(scan_path_frame, "Amplitude", 6, PHI_AMPLITUDE_SET_CMD, "phi_amplitude")
        self.phi_centre_entry = self._add_pid_row(scan_path_frame, "Centre Offset", 7, PHI_CENTRE_SET_CMD, "phi_centre")

        # Targets
        target_frame = ttk.LabelFrame(right_column, text="Motor Targets", padding=12)
        target_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        
        target_row1 = ttk.Frame(target_frame)
        target_row1.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        ttk.Label(target_row1, text="Motor 1:", font=("Arial", 9)).grid(row=0, column=0, padx=(0, 10), sticky="w")
        self.target_entry = ttk.Entry(target_row1, width=18)
        self.target_entry.insert(0, str(DEFAULTS.get("target_1", 0.0)))
        self.target_entry.grid(row=0, column=1, padx=5)
        ttk.Button(target_row1, text="Set", command=self.send_target, width=10).grid(row=0, column=2, padx=5)
        target_row1.grid_columnconfigure(1, weight=1)
        
        target_row2 = ttk.Frame(target_frame)
        target_row2.grid(row=1, column=0, columnspan=3, sticky="ew")
        ttk.Label(target_row2, text="Motor 2:", font=("Arial", 9)).grid(row=0, column=0, padx=(0, 10), sticky="w")
        self.target_2_entry = ttk.Entry(target_row2, width=18)
        self.target_2_entry.insert(0, str(DEFAULTS.get("target_2", 0.0)))
        self.target_2_entry.grid(row=0, column=1, padx=5)
        ttk.Button(target_row2, text="Set", command=self.send_target_2, width=10).grid(row=0, column=2, padx=5)
        target_row2.grid_columnconfigure(1, weight=1)

        # LDM Configuration
        ldm_frame = ttk.LabelFrame(right_column, text="LDM Configuration", padding=12)
        ldm_frame.grid(row=2, column=0, sticky="ew")
        
        ldm_row = ttk.Frame(ldm_frame)
        ldm_row.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 5))
        ttk.Label(ldm_row, text="Preset:", font=("Arial", 9)).grid(row=0, column=0, padx=(0, 10), sticky="w")
        self.ldm_config_entry = ttk.Entry(ldm_row, width=18)
        self.ldm_config_entry.insert(0, str(DEFAULTS.get("ldm_config", 0.0)))
        self.ldm_config_entry.grid(row=0, column=1, padx=5)
        ttk.Button(ldm_row, text="Reconfigure", command=self.send_ldm_config, width=15).grid(row=0, column=2, padx=5)
        ldm_row.grid_columnconfigure(1, weight=1)
        
        ttk.Label(
            ldm_frame, 
            text="⚠ System must be disabled before reconfiguration", 
            font=("Arial", 8), 
            foreground="orange"
        ).grid(row=1, column=0, columnspan=3, pady=(5, 0))

        settings_monitor_frame = ttk.LabelFrame(main_frame, text="Settings Monitor", padding=5)
        settings_monitor_frame.grid(row=0, column=2, pady=10, sticky="nsew", rowspan=3, columnspan=3)
        settings_monitor_frame.grid_rowconfigure(1, weight=1)
        settings_monitor_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(1, weight=1)

        ttk.Button(settings_monitor_frame, text="Clear Monitor", command=self.clear_monitor).grid(
            row=0, column=0, sticky="e", pady=2
        )
        self.settings_serial_text_data = scrolledtext.ScrolledText(
            settings_monitor_frame, height=15, width=80, bg="black", fg="lime", font=("Courier", 9)
        )
        self.settings_serial_text_data.grid(row=1, column=0, sticky="nsew")


    def setup_data_tab(self):
        """Setup the Incoming Data Dashboard tab."""
        self.data_tab.grid_rowconfigure(0, weight=1)
        self.data_tab.grid_columnconfigure(0, weight=1)

        main_frame = ttk.Frame(self.data_tab)
        main_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        main_frame.grid_rowconfigure(1, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)

        # Sensor Data Display
        sensor_frame = ttk.LabelFrame(main_frame, text="Live Sensor Data", padding=15)
        sensor_frame.grid(row=0, column=0, padx=5, pady=5, sticky="new")
        sensor_frame.grid_columnconfigure(1, weight=1)
        sensor_frame.grid_columnconfigure(3, weight=1)
        
        # Distance
        ttk.Label(sensor_frame, text="Distance (cm):", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=10, pady=8, sticky="e")
        self.distance_var = tk.StringVar(value="---")
        ttk.Label(sensor_frame, textvariable=self.distance_var, font=("Courier", 12, "bold"), width=12).grid(
            row=0, column=1, padx=10, pady=8, sticky="w"
        )
        
        # Packet rate
        ttk.Label(sensor_frame, text="Packets/sec:", font=("Arial", 10, "bold")).grid(row=0, column=2, padx=10, pady=8, sticky="e")
        self.packet_rate_var = tk.StringVar(value="0")
        ttk.Label(sensor_frame, textvariable=self.packet_rate_var, font=("Courier", 12, "bold"), width=12).grid(
            row=0, column=3, padx=10, pady=8, sticky="w"
        )
        
        # Recording controls
        record_frame = ttk.Frame(sensor_frame)
        record_frame.grid(row=1, column=0, columnspan=4, sticky="ew", pady=10)
        record_frame.grid_columnconfigure(1, weight=1)
        
        self.record_btn = ttk.Button(
            record_frame, 
            text="Start Recording", 
            command=self.toggle_recording,
            width=18
        )
        self.record_btn.grid(row=0, column=0, padx=10, pady=5)
        
        self.record_status_var = tk.StringVar(value="Not Recording")
        self.record_status_label = ttk.Label(
            record_frame, 
            textvariable=self.record_status_var, 
            font=("Arial", 9),
            foreground="gray"
        )
        self.record_status_label.grid(row=0, column=1, padx=10, pady=5, sticky="w")
        
        self.record_count_var = tk.StringVar(value="Packets: 0")
        ttk.Label(
            record_frame, 
            textvariable=self.record_count_var, 
            font=("Courier", 9),
            foreground="gray"
        ).grid(row=0, column=2, padx=10, pady=5, sticky="e")
        
        # Separator
        ttk.Separator(sensor_frame, orient="horizontal").grid(row=2, column=0, columnspan=4, sticky="ew", pady=10)
        
        # Motor 1 data
        ttk.Label(sensor_frame, text="Motor 1:", font=("Arial", 10, "bold")).grid(row=3, column=0, padx=10, pady=5, sticky="e")
        ttk.Label(sensor_frame, text="Angle (rad):").grid(row=4, column=0, padx=10, pady=3, sticky="e")
        self.angle1_var = tk.StringVar(value="---")
        ttk.Label(sensor_frame, textvariable=self.angle1_var, font=("Courier", 11, "bold"), width=12).grid(
            row=4, column=1, padx=10, pady=3, sticky="w"
        )
        ttk.Label(sensor_frame, text="Target (rad):").grid(row=4, column=2, padx=10, pady=3, sticky="e")
        self.target1_var = tk.StringVar(value="---")
        ttk.Label(sensor_frame, textvariable=self.target1_var, font=("Courier", 11, "bold"), width=12).grid(
            row=4, column=3, padx=10, pady=3, sticky="w"
        )
        
        # Motor 2 data
        ttk.Label(sensor_frame, text="Motor 2:", font=("Arial", 10, "bold")).grid(row=5, column=0, padx=10, pady=5, sticky="e")
        ttk.Label(sensor_frame, text="Angle (rad):").grid(row=6, column=0, padx=10, pady=3, sticky="e")
        self.angle2_var = tk.StringVar(value="---")
        ttk.Label(sensor_frame, textvariable=self.angle2_var, font=("Courier", 11, "bold"), width=12).grid(
            row=6, column=1, padx=10, pady=3, sticky="w"
        )
        ttk.Label(sensor_frame, text="Target (rad):").grid(row=6, column=2, padx=10, pady=3, sticky="e")
        self.target2_var = tk.StringVar(value="---")
        ttk.Label(sensor_frame, textvariable=self.target2_var, font=("Courier", 11, "bold"), width=12).grid(
            row=6, column=3, padx=10, pady=3, sticky="w"
        )
        
        self.packet_count = 0
        self.last_rate_time = time.time()

        # Serial Monitor for Data tab
        monitor_frame = ttk.LabelFrame(main_frame, text="Serial Monitor", padding=5)
        monitor_frame.grid(row=1, column=0, pady=10, sticky="nsew")
        monitor_frame.grid_rowconfigure(1, weight=1)
        monitor_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(1, weight=1)

        ttk.Button(monitor_frame, text="Clear Monitor", command=self.clear_monitor).grid(
            row=0, column=0, sticky="e", pady=2
        )
        self.serial_text_data = scrolledtext.ScrolledText(
            monitor_frame, height=15, width=80, bg="black", fg="lime", font=("Courier", 9)
        )
        self.serial_text_data.grid(row=1, column=0, sticky="nsew")

    def _add_pid_row(self, frame, label, row, command, default_key):
        ttk.Label(frame, text=f"{label}:", font=("Arial", 9)).grid(
            row=row, column=0, padx=(0, 10), pady=4, sticky="e"
        )
        entry = ttk.Entry(frame, width=14, font=("Courier", 9))
        entry.insert(0, str(DEFAULTS.get(default_key, 0.0)))
        entry.grid(row=row, column=1, padx=5, pady=4, sticky="w")
        ttk.Button(
            frame, 
            text="Set", 
            command=lambda e=entry, cmd=command: self.send_pid_value(e, cmd),
            width=8
        ).grid(row=row, column=2, padx=(5, 0), pady=4)
        frame.grid_columnconfigure(1, weight=1)
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

        # Stop recording if active
        if self.is_recording:
            self.stop_recording()

        self.is_connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.log_message("=== Disconnected ===\n")
        
        # Reset sensor display
        self.distance_var.set("---")
        self.angle1_var.set("---")
        self.angle2_var.set("---")
        self.target1_var.set("---")
        self.target2_var.set("---")
        self.packet_rate_var.set("0")
        self.packet_count = 0

    def read_serial(self):
        """
        Read serial data, parsing both text output and binary DataPackets.
        Uses a buffer-based state machine for efficient mixed-mode parsing.
        Optimized for high-throughput data handling.
        """
        buffer = bytearray()
        buffer_pos = 0  # Track position to avoid repeated slicing
        
        # Local references for faster attribute access in hot loop
        data_queue_put = self.data_queue.put
        message_queue_put = self.message_queue.put
        unpack = DATA_PACKET_STRUCT.unpack_from
        
        while self.running and self.serial_port:
            try:
                # Read all available bytes at once
                waiting = self.serial_port.in_waiting
                if waiting > 0:
                    buffer.extend(self.serial_port.read(waiting))
                
                buffer_len = len(buffer)
                
                # Process buffer
                while buffer_pos < buffer_len:
                    # Check for binary packet (starts with SEND_DATA_CMD)
                    if buffer[buffer_pos] == SEND_DATA_CMD:
                        remaining = buffer_len - buffer_pos
                        if remaining >= DATA_PACKET_SIZE:
                            # Parse packet directly from buffer using unpack_from (zero-copy)
                            try:
                                _, distance, angle1, angle2, target1, target2 = unpack(buffer, buffer_pos)
                                data_queue_put((distance, angle1, angle2, target1, target2))
                                
                                # Buffer packet for recording (batched writes)
                                if self.is_recording:
                                    self.record_buffer.extend(buffer[buffer_pos:buffer_pos + DATA_PACKET_SIZE])
                                    self.record_buffer_size += DATA_PACKET_SIZE
                                    self.recorded_packets += 1
                                    
                                    # Flush buffer to disk when threshold reached
                                    if self.record_buffer_size >= self.record_flush_threshold:
                                        self._flush_record_buffer()
                                
                                buffer_pos += DATA_PACKET_SIZE
                            except struct.error:
                                # Malformed packet, skip one byte
                                buffer_pos += 1
                        else:
                            # Incomplete packet, wait for more data
                            break
                    else:
                        # Text data - find end boundary
                        search_start = buffer_pos
                        newline_idx = buffer.find(b'\n', search_start)
                        packet_idx = buffer.find(SEND_DATA_CMD_BYTES, search_start + 1)
                        
                        if newline_idx >= 0 and (packet_idx < 0 or newline_idx < packet_idx):
                            end_idx = newline_idx + 1
                        elif packet_idx >= 0:
                            end_idx = packet_idx
                        elif buffer_len - buffer_pos > 512:
                            end_idx = buffer_len
                        else:
                            break
                        
                        text = buffer[buffer_pos:end_idx].decode("utf-8", errors="ignore")
                        buffer_pos = end_idx
                        
                        if text.strip():
                            message_queue_put(text if text.endswith('\n') else text + '\n')
                
                # Compact buffer: remove processed bytes
                if buffer_pos > 0:
                    del buffer[:buffer_pos]
                    buffer_pos = 0
                
            except Exception as e:
                if self.running:
                    message_queue_put(f"Read error: {str(e)}\n")
                break
            
            time.sleep(0.001)  # Short sleep for responsiveness
        
        # Flush any remaining recording data on exit
        if self.is_recording and self.record_buffer_size > 0:
            self._flush_record_buffer()
    
    def _flush_record_buffer(self):
        """Flush the recording buffer to disk."""
        if self.record_file and self.record_buffer_size > 0:
            try:
                self.record_file.write(self.record_buffer)
                self.record_buffer.clear()
                self.record_buffer_size = 0
            except Exception as e:
                self.message_queue.put(f"Recording flush error: {str(e)}\n")
                self.stop_recording()

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
        value = self._read_float(self.target_entry, "target_1")
        if value is not None:
            self.send_packet(TARGET_1_SET_CMD, value)

    def send_target_2(self):
        value = self._read_float(self.target_2_entry, "target_2")
        if value is not None:
            self.send_packet(TARGET_2_SET_CMD, value)

    def send_ldm_config(self):
        value = self._read_float(self.ldm_config_entry, "ldm_config")
        if value is not None:
            self.send_packet(RECONFIGURE_LDM_CMD, value)

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
        # Batch process text messages (limit per cycle to keep UI responsive)
        for _ in range(100):
            if self.message_queue.empty():
                break
            message = self.message_queue.get_nowait()
            # Update serial monitor in both tabs
            self.serial_text_data.insert(tk.END, message)
            self.settings_serial_text_data.insert(tk.END, message)
        
        self.serial_text_data.see(tk.END)
        self.settings_serial_text_data.see(tk.END)
        self.root.after(50, self.update_serial_monitor)
    
    def update_sensor_display(self):
        """Process incoming sensor data packets and update display."""
        packets_this_cycle = 0
        latest_data = None
        data_queue = self.data_queue  # Local reference for speed
        data_history_append = self.data_history.append
        
        # Drain the queue efficiently, keeping only the latest reading for display
        try:
            while True:
                latest_data = data_queue.get_nowait()
                data_history_append(latest_data)
                packets_this_cycle += 1
        except:
            pass  # Queue empty
        
        # Update display with latest values (only if we have data)
        if latest_data is not None:
            distance, angle1, angle2, target1, target2 = latest_data
            self.distance_var.set(str(distance))
            self.angle1_var.set(f"{angle1:.3f}")
            self.angle2_var.set(f"{angle2:.3f}")
            self.target1_var.set(f"{target1:.3f}")
            self.target2_var.set(f"{target2:.3f}")
        
        # Update packet rate counter
        self.packet_count += packets_this_cycle
        now = time.time()
        elapsed = now - self.last_rate_time
        if elapsed >= 1.0:
            rate = self.packet_count / elapsed
            self.packet_rate_var.set(f"{rate:.1f}")
            self.packet_count = 0
            self.last_rate_time = now
        
        # Update recording count display (only when recording)
        if self.is_recording:
            self.record_count_var.set(f"Packets: {self.recorded_packets}")
        
        self.root.after(16, self.update_sensor_display)  # ~60 Hz UI update

    def log_message(self, message):
        # Log to serial monitor in both tabs
        self.serial_text_data.insert(tk.END, message)
        self.settings_serial_text_data.insert(tk.END, message)
        self.serial_text_data.see(tk.END)
        self.settings_serial_text_data.see(tk.END)

    def clear_monitor(self):
        # Clear serial monitor in both tabs
        self.serial_text_data.delete(1.0, tk.END)
        self.settings_serial_text_data.delete(1.0, tk.END)

    def toggle_recording(self):
        """Start or stop recording data packets."""
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        """Start recording incoming data packets to file."""
        if not self.is_connected:
            self.log_message("Error: Must be connected to start recording\n")
            return
        
        # Ensure data directory exists
        data_dir = os.path.join(os.path.dirname(__file__), "data")
        os.makedirs(data_dir, exist_ok=True)
        
        file_path = os.path.join(data_dir, "raw_data.bin")
        
        try:
            # Open file in binary write mode (overwrites existing file)
            self.record_file = open(file_path, "wb")
            self.is_recording = True
            self.recorded_packets = 0
            
            # Update UI
            self.record_btn.config(text="Stop Recording")
            self.record_status_var.set("Recording...")
            self.record_status_label.config(foreground="red")
            self.record_count_var.set("Packets: 0")
            
            self.log_message(f"=== Started recording to {file_path} ===\n")
        except Exception as e:
            self.log_message(f"Error starting recording: {str(e)}\n")
            self.is_recording = False
            if self.record_file:
                self.record_file.close()
                self.record_file = None

    def stop_recording(self):
        """Stop recording and close the file."""
        if not self.is_recording:
            return
        
        try:
            # Flush any remaining buffered data
            if self.record_buffer_size > 0:
                self._flush_record_buffer()
            
            if self.record_file:
                self.record_file.close()
                self.record_file = None
            
            self.is_recording = False
            self.record_buffer.clear()
            self.record_buffer_size = 0
            
            # Update UI
            self.record_btn.config(text="Start Recording")
            self.record_status_var.set("Not Recording")
            self.record_status_label.config(foreground="gray")
            
            self.log_message(f"=== Stopped recording: {self.recorded_packets} packets saved ===\n")
        except Exception as e:
            self.log_message(f"Error stopping recording: {str(e)}\n")


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
