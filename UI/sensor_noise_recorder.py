"""
Sensor Noise Recorder UI
Tkinter applet that receives binary sensor data packets from serial
at ~5KHz, displays live readout, serial monitor, and can record data to a binary file.
"""

import struct
import threading
import time
import tkinter as tk
from tkinter import scrolledtext, ttk, filedialog, messagebox
import os
import serial
import serial.tools.list_ports
from datetime import datetime
from pathlib import Path
from queue import Queue

# Data packet format: command token (0xAB) + float + float + float + uint16 + uint16 (little-endian)
DATA_PACKET_CMD = 0xAB
PACKET_SIZE = 17  # 1 (cmd) + 4 (float) + 4 (float) + 4 (float) + 2 (uint16) + 2 (uint16)
PACKET_FORMAT = "<BfffHH"  # little-endian: uint8, float, float, float, uint16, uint16
PACKET_STRUCT = struct.Struct(PACKET_FORMAT)

# Command format: uint8_t cmd + float data (little-endian)
KALMAN_QA_SET_CMD = 0b00010011  # 19 in decimal
LPF_VELOCITY_TAU_SET_CMD = 0b00010010  # 18 in decimal
LPF_ANGLE_TAU_SET_CMD = 0b00010011  # 19 in decimal
NOTCH_FILTER_FREQ_SET_CMD = 0b11010011
NOTCH_FILTER_Q_SET_CMD = 0b11010100
NOTCH_FILTER_SAMPLE_RATE_SET_CMD = 0b11010101
CMD_FORMAT = "<Bf"  # little-endian: uint8, float
CMD_STRUCT = struct.Struct(CMD_FORMAT)

# Default serial settings
DEFAULT_BAUD_RATE = 460800


class SensorNoiseRecorderUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Sensor Noise Recorder")
        self.root.geometry("1000x700")
        
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        # Bounded queue for display updates (drains every 50ms, so max ~2500 packets at 50KHz)
        # Using maxsize as safety measure - if full, oldest packets are dropped (recording unaffected)
        self.data_queue = Queue(maxsize=5000)  # Thread-safe queue for data packets
        self.message_queue = Queue()  # Thread-safe queue for text messages
        
        # Recording state
        self.is_recording = False
        self.record_file = None
        self.recorded_packets = 0
        self.record_start_time = None
        
        # Statistics
        self.total_packets_received = 0
        self.last_update_time = time.time()
        self.packets_since_last_update = 0
        
        # Latest values for display
        self.latest_raw = None
        self.latest_angle = None
        self.latest_velocity = None
        self.latest_magnitude = None
        self.latest_agc = None
        
        # Counter for displaying every 100th value
        self.display_counter = 0
        
        self.setup_ui()
        
    def setup_ui(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Serial Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        ttk.Button(conn_frame, text="Connect", command=self.connect_serial).grid(row=0, column=3, padx=5)
        ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_serial).grid(row=0, column=4, padx=5)
        
        # Kalman Qa control frame
        # kalman_frame = ttk.LabelFrame(main_frame, text="Kalman Qa Control", padding="5")
        # kalman_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # ttk.Label(kalman_frame, text="Qa value:").grid(row=0, column=0, padx=5)
        # self.qa_var = tk.StringVar(value="0.001")
        # qa_entry = ttk.Entry(kalman_frame, textvariable=self.qa_var, width=15)
        # qa_entry.grid(row=0, column=1, padx=5)
        
        # ttk.Button(kalman_frame, text="Set Kalman Qa", command=self.send_kalman_qa_cmd).grid(row=0, column=2, padx=5)
        
        # LPF Velocity Tau control frame
        lpf_frame = ttk.LabelFrame(main_frame, text="LPF Control", padding="5")
        lpf_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(lpf_frame, text="Velocity Tau value:").grid(row=0, column=0, padx=5)
        self.velocity_tau_var = tk.StringVar(value="0.01")
        velocity_tau_entry = ttk.Entry(lpf_frame, textvariable=self.velocity_tau_var, width=15)
        velocity_tau_entry.grid(row=0, column=1, padx=5)
        ttk.Button(lpf_frame, text="Set LPF Velocity Tau", command=self.send_lpf_velocity_tau_cmd).grid(row=0, column=2, padx=5)

        ttk.Label(lpf_frame, text="Angle Tau value:").grid(row=1, column=0, padx=5)
        self.angle_tau_var = tk.StringVar(value="0.0001")
        angle_tau_entry = ttk.Entry(lpf_frame, textvariable=self.angle_tau_var, width=15)
        angle_tau_entry.grid(row=1, column=1, padx=5)
        ttk.Button(lpf_frame, text="Set LPF Angle Tau", command=self.send_lpf_angle_tau_cmd).grid(row=1, column=2, padx=5)
        
        # Notch Filter Frequency control frame
        # notch_freq_frame = ttk.LabelFrame(main_frame, text="Notch Filter Frequency Control", padding="5")
        # notch_freq_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # ttk.Label(notch_freq_frame, text="Frequency (Hz):").grid(row=0, column=0, padx=5)
        # self.notch_freq_var = tk.StringVar(value="50.0")
        # notch_freq_entry = ttk.Entry(notch_freq_frame, textvariable=self.notch_freq_var, width=15)
        # notch_freq_entry.grid(row=0, column=1, padx=5)
        
        # ttk.Button(notch_freq_frame, text="Set Notch Frequency", command=self.send_notch_filter_freq_cmd).grid(row=0, column=2, padx=5)
        
        # # Notch Filter Q control frame
        # notch_q_frame = ttk.LabelFrame(main_frame, text="Notch Filter Q Control", padding="5")
        # notch_q_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # ttk.Label(notch_q_frame, text="Q value:").grid(row=0, column=0, padx=5)
        # self.notch_q_var = tk.StringVar(value="1.0")
        # notch_q_entry = ttk.Entry(notch_q_frame, textvariable=self.notch_q_var, width=15)
        # notch_q_entry.grid(row=0, column=1, padx=5)
        
        # ttk.Button(notch_q_frame, text="Set Notch Q", command=self.send_notch_filter_q_cmd).grid(row=0, column=2, padx=5)
        
        # # Notch Filter Sample Rate control frame
        # notch_sr_frame = ttk.LabelFrame(main_frame, text="Notch Filter Sample Rate Control", padding="5")
        # notch_sr_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # ttk.Label(notch_sr_frame, text="Sample Rate (Hz):").grid(row=0, column=0, padx=5)
        # self.notch_sr_var = tk.StringVar(value="10000.0")
        # notch_sr_entry = ttk.Entry(notch_sr_frame, textvariable=self.notch_sr_var, width=15)
        # notch_sr_entry.grid(row=0, column=1, padx=5)
        
        # ttk.Button(notch_sr_frame, text="Set Notch Sample Rate", command=self.send_notch_filter_sample_rate_cmd).grid(row=0, column=2, padx=5)
        
        # Status frame
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="5")
        status_frame.grid(row=6, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=0, padx=5)
        
        self.packet_count_label = ttk.Label(status_frame, text="Packets: 0")
        self.packet_count_label.grid(row=0, column=1, padx=5)
        
        self.rate_label = ttk.Label(status_frame, text="Rate: 0 Hz")
        self.rate_label.grid(row=0, column=2, padx=5)
        
        # Create notebook for tabs
        notebook = ttk.Notebook(main_frame)
        notebook.grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        main_frame.rowconfigure(7, weight=1)
        main_frame.columnconfigure(0, weight=1)
        
        # Serial Monitor tab
        monitor_frame = ttk.Frame(notebook, padding="5")
        notebook.add(monitor_frame, text="Serial Monitor")
        monitor_frame.columnconfigure(0, weight=1)
        monitor_frame.rowconfigure(0, weight=1)
        
        self.monitor_text = scrolledtext.ScrolledText(monitor_frame, height=15, width=80, state=tk.DISABLED)
        self.monitor_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Live Data Readout tab
        readout_frame = ttk.Frame(notebook, padding="5")
        notebook.add(readout_frame, text="Live Data Readout")
        readout_frame.columnconfigure(0, weight=1)
        readout_frame.rowconfigure(0, weight=1)
        
        self.readout_text = scrolledtext.ScrolledText(readout_frame, height=15, width=80, state=tk.DISABLED)
        self.readout_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Recording frame
        record_frame = ttk.LabelFrame(main_frame, text="Recording", padding="5")
        record_frame.grid(row=8, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.record_button = ttk.Button(record_frame, text="Start Recording", command=self.toggle_recording)
        self.record_button.grid(row=0, column=0, padx=5)
        
        self.record_status_label = ttk.Label(record_frame, text="Not recording", foreground="gray")
        self.record_status_label.grid(row=0, column=1, padx=5)
        
        self.recorded_count_label = ttk.Label(record_frame, text="Recorded: 0 packets")
        self.recorded_count_label.grid(row=0, column=2, padx=5)
        
        # Latest values display
        latest_frame = ttk.LabelFrame(main_frame, text="Latest Values", padding="5")
        latest_frame.grid(row=9, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.raw_label = ttk.Label(latest_frame, text="Raw: ---", font=("Arial", 12))
        self.raw_label.grid(row=0, column=0, padx=10)
        
        self.angle_label = ttk.Label(latest_frame, text="Angle: --- rad", font=("Arial", 12))
        self.angle_label.grid(row=0, column=1, padx=10)
        
        self.velocity_label = ttk.Label(latest_frame, text="Velocity: --- rad/s", font=("Arial", 12))
        self.velocity_label.grid(row=0, column=2, padx=10)
        
        self.magnitude_label = ttk.Label(latest_frame, text="Magnitude: ---", font=("Arial", 12))
        self.magnitude_label.grid(row=0, column=3, padx=10)
        
        self.agc_label = ttk.Label(latest_frame, text="AGC: ---", font=("Arial", 12))
        self.agc_label.grid(row=0, column=4, padx=10)
        
        # Refresh ports on startup
        self.refresh_ports()
        
        # Start UI update timer
        self.update_ui()
        
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
    
    def connect_serial(self):
        if self.is_connected:
            return
        
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a serial port")
            return
        
        try:
            # Create serial port without opening, disable DTR to prevent Teensy reset
            self.serial_port = serial.Serial()
            self.serial_port.port = port
            self.serial_port.baudrate = DEFAULT_BAUD_RATE
            self.serial_port.timeout = 0.1
            self.serial_port.dtr = False  # Prevent DTR from resetting the Teensy
            self.serial_port.open()
            self.is_connected = True
            self.running = True
            self.status_label.config(text=f"Connected: {port}", foreground="green")
            
            # Clear previous data
            self.monitor_text.config(state=tk.NORMAL)
            self.monitor_text.delete("1.0", tk.END)
            self.monitor_text.config(state=tk.DISABLED)
            
            self.readout_text.config(state=tk.NORMAL)
            self.readout_text.delete("1.0", tk.END)
            self.readout_text.config(state=tk.DISABLED)
            
            # Start read thread
            self.read_thread = threading.Thread(target=self.read_serial_thread, daemon=True)
            self.read_thread.start()
            
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            self.is_connected = False
    
    def disconnect_serial(self):
        if not self.is_connected:
            return
        
        self.running = False
        if self.is_recording:
            self.toggle_recording()  # Stop recording if active
        
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        
        self.is_connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.total_packets_received = 0
        self.packets_since_last_update = 0
    
    def send_kalman_qa_cmd(self):
        """Send KALMAN_QA_SET_CMD command with the specified Qa value"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        try:
            qa_value = float(self.qa_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid Qa value. Please enter a valid number.")
            return
        
        try:
            # Pack command: uint8_t cmd + float data (little-endian)
            cmd_bytes = CMD_STRUCT.pack(KALMAN_QA_SET_CMD, qa_value)
            self.serial_port.write(cmd_bytes)
            self.serial_port.flush()
            print(f"Sent KALMAN_QA_SET_CMD with Qa={qa_value}")
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Unexpected error: {e}")
    
    def send_lpf_velocity_tau_cmd(self):
        """Send LPF_VELOCITY_TAU_SET_CMD command with the specified tau value"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        try:
            tau_value = float(self.velocity_tau_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid tau value. Please enter a valid number.")
            return
        
        try:
            # Pack command: uint8_t cmd + float data (little-endian)
            cmd_bytes = CMD_STRUCT.pack(LPF_VELOCITY_TAU_SET_CMD, tau_value)
            self.serial_port.write(cmd_bytes)
            self.serial_port.flush()
            print(f"Sent LPF_VELOCITY_TAU_SET_CMD with tau={tau_value}")
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Unexpected error: {e}")

    def send_lpf_angle_tau_cmd(self):
        """Send LPF_ANGLE_TAU_SET_CMD command with the specified tau value"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        try:
            tau_value = float(self.angle_tau_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid tau value. Please enter a valid number.")
            return
        
        try:
            # Pack command: uint8_t cmd + float data (little-endian)
            cmd_bytes = CMD_STRUCT.pack(LPF_ANGLE_TAU_SET_CMD, tau_value)
            self.serial_port.write(cmd_bytes)
            self.serial_port.flush()
            print(f"Sent LPF_ANGLE_TAU_SET_CMD with tau={tau_value}")
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Unexpected error: {e}")
    
    def send_notch_filter_freq_cmd(self):
        """Send NOTCH_FILTER_FREQ_SET_CMD command with the specified frequency value"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        try:
            freq_value = float(self.notch_freq_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid frequency value. Please enter a valid number.")
            return
        
        try:
            # Pack command: uint8_t cmd + float data (little-endian)
            cmd_bytes = CMD_STRUCT.pack(NOTCH_FILTER_FREQ_SET_CMD, freq_value)
            self.serial_port.write(cmd_bytes)
            self.serial_port.flush()
            print(f"Sent NOTCH_FILTER_FREQ_SET_CMD with frequency={freq_value}")
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Unexpected error: {e}")
    
    def send_notch_filter_q_cmd(self):
        """Send NOTCH_FILTER_Q_SET_CMD command with the specified Q value"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        try:
            q_value = float(self.notch_q_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid Q value. Please enter a valid number.")
            return
        
        try:
            # Pack command: uint8_t cmd + float data (little-endian)
            cmd_bytes = CMD_STRUCT.pack(NOTCH_FILTER_Q_SET_CMD, q_value)
            self.serial_port.write(cmd_bytes)
            self.serial_port.flush()
            print(f"Sent NOTCH_FILTER_Q_SET_CMD with Q={q_value}")
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Unexpected error: {e}")
    
    def send_notch_filter_sample_rate_cmd(self):
        """Send NOTCH_FILTER_SAMPLE_RATE_SET_CMD command with the specified sample rate value"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        try:
            sr_value = float(self.notch_sr_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid sample rate value. Please enter a valid number.")
            return
        
        try:
            # Pack command: uint8_t cmd + float data (little-endian)
            cmd_bytes = CMD_STRUCT.pack(NOTCH_FILTER_SAMPLE_RATE_SET_CMD, sr_value)
            self.serial_port.write(cmd_bytes)
            self.serial_port.flush()
            print(f"Sent NOTCH_FILTER_SAMPLE_RATE_SET_CMD with sample_rate={sr_value}")
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
        except Exception as e:
            messagebox.showerror("Error", f"Unexpected error: {e}")
    
    def read_serial_thread(self):
        """Background thread that reads serial data"""
        buffer = bytearray()
        incomplete_text = bytearray()  # Buffer for incomplete text messages
        
        while self.running and self.is_connected:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                    
                    # Process buffer: look for packets and text messages
                    while len(buffer) > 0:
                        # Look for command token
                        cmd_idx = buffer.find(DATA_PACKET_CMD)
                        
                        if cmd_idx == -1:
                            # No command token found, treat all as text
                            if len(buffer) > 0:
                                incomplete_text.extend(buffer)
                                buffer.clear()
                                # Process complete text lines, keeping incomplete in buffer
                                incomplete_text = self._process_text_buffer(incomplete_text, keep_incomplete=True)
                            break
                        
                        # Process text before the command token
                        if cmd_idx > 0:
                            incomplete_text.extend(buffer[:cmd_idx])
                            buffer = buffer[cmd_idx:]
                            # Process complete text lines, keeping incomplete in buffer
                            incomplete_text = self._process_text_buffer(incomplete_text, keep_incomplete=True)
                        
                        # Check if we have a complete packet starting at position 0
                        if len(buffer) >= PACKET_SIZE:
                            packet_bytes = buffer[:PACKET_SIZE]
                            buffer = buffer[PACKET_SIZE:]
                            
                            try:
                                # Unpack packet: uint8_t cmd, float raw, float angle, float velocity, uint16 magnitude, uint16 agc
                                cmd, raw_value, rotor_angle, angular_velocity, sensor_magnitude, agc_value = PACKET_STRUCT.unpack(packet_bytes)
                                
                                if cmd == DATA_PACKET_CMD:
                                    # Valid packet
                                    packet_data = (raw_value, rotor_angle, angular_velocity, sensor_magnitude, agc_value, time.time())
                                    
                                    # Add to display queue (non-blocking - drops oldest if full)
                                    # This doesn't affect recording, which happens immediately below
                                    try:
                                        self.data_queue.put_nowait(packet_data)
                                    except:
                                        # Queue full - drop oldest and add new (display only, recording unaffected)
                                        try:
                                            self.data_queue.get_nowait()
                                            self.data_queue.put_nowait(packet_data)
                                        except:
                                            pass  # Queue operations failed, skip display update for this packet
                                    
                                    self.total_packets_received += 1
                                    self.packets_since_last_update += 1
                                    
                                    # Write to file if recording (this happens immediately, no queue lag)
                                    # Recording is completely independent of display queue
                                    if self.is_recording and self.record_file:
                                        try:
                                            if not self.record_file.closed:
                                                self.record_file.write(packet_bytes)
                                                self.record_file.flush()
                                                self.recorded_packets += 1
                                        except (ValueError, OSError) as e:
                                            print(f"File write error: {e}")
                                            self.is_recording = False
                                else:
                                    # Invalid command token, treat as text and continue
                                    incomplete_text.append(cmd)
                                    
                            except struct.error as e:
                                print(f"Unpack error: {e}")
                                # Skip the command byte and try again
                                if len(buffer) > 0:
                                    buffer = buffer[1:]
                        else:
                            # Not enough data for a complete packet, wait for more
                            break
                    
                    # Prevent text buffer from growing too large
                    if len(incomplete_text) > 1000:
                        incomplete_text.clear()
                
                time.sleep(0.001)  # Small delay to prevent tight loop
                
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.running = False
                break
            except Exception as e:
                print(f"Unexpected error in read thread: {e}")
                break
    
    def _process_text_buffer(self, text_buffer, keep_incomplete=False):
        """Process text buffer and extract complete messages.
        
        Returns:
            bytearray: Remaining incomplete text (if keep_incomplete=True), or empty bytearray
        """
        if not text_buffer:
            return bytearray()
        
        # Split by newlines
        lines = text_buffer.split(b'\n')
        
        # Process complete lines (everything except the last, which might be incomplete)
        for line in lines[:-1]:
            if line.strip():
                try:
                    message = line.decode('utf-8', errors='ignore').strip()
                    if message:
                        self.message_queue.put(message)
                except:
                    pass
        
        # Handle incomplete line
        if keep_incomplete:
            # Return the last incomplete line
            if lines and lines[-1]:
                return bytearray(lines[-1])
            else:
                return bytearray()
        else:
            # Process the last line if it ends with \r
            if lines and lines[-1]:
                if lines[-1].endswith(b'\r'):
                    try:
                        message = lines[-1].decode('utf-8', errors='ignore').strip()
                        if message:
                            self.message_queue.put(message)
                    except:
                        pass
            return bytearray()
    
    def toggle_recording(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        if not self.is_recording:
            # Start recording
            filename = "alpha_beta.bin"
            # Use absolute path based on script location
            script_dir = Path(__file__).parent.absolute()
            data_dir = script_dir / "data"
            filepath = data_dir / filename
            
            # Ensure data directory exists
            data_dir.mkdir(exist_ok=True)
            
            try:
                # Ensure file is closed if it was previously open
                if self.record_file:
                    self.record_file.close()
                    self.record_file = None
                
                # Delete existing file to ensure it's completely empty
                if filepath.exists():
                    filepath.unlink()
                
                # Open file in write mode
                self.record_file = open(filepath, "wb")
                self.record_file.flush()
                print(f"Created recording file: {filepath}")
                
                self.is_recording = True
                self.recorded_packets = 0
                self.record_start_time = time.time()
                self.record_button.config(text="Stop Recording")
                self.record_status_label.config(text=f"Recording: {filename}", foreground="red")
                self.recorded_count_label.config(text="Recorded: 0 packets")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to open file: {e}\nPath: {filepath}")
                print(f"Error creating file: {e}")
                print(f"Attempted path: {filepath}")
                self.is_recording = False
        else:
            # Stop recording
            self.is_recording = False
            
            filepath = None
            if self.record_file:
                filepath = Path(self.record_file.name)
            
            if self.record_file:
                try:
                    self.record_file.flush()
                    self.record_file.close()
                    print(f"Closed recording file: {filepath}")
                    if filepath and filepath.exists():
                        print(f"File exists after close: {filepath}, size: {filepath.stat().st_size} bytes")
                except Exception as e:
                    print(f"Error closing file: {e}")
                self.record_file = None
            
            self.record_button.config(text="Start Recording")
            self.record_status_label.config(text="Not recording", foreground="gray")
            
            duration = time.time() - self.record_start_time if self.record_start_time else 0
            file_info = ""
            if filepath and filepath.exists():
                file_info = f"\nFile: {filepath}\nFile size: {filepath.stat().st_size} bytes"
            messagebox.showinfo("Recording Complete", 
                              f"Recorded {self.recorded_packets} packets\n"
                              f"Duration: {duration:.2f} seconds\n"
                              f"Average rate: {self.recorded_packets/duration:.1f} Hz" if duration > 0 else ""
                              + file_info)
    
    def update_ui(self):
        """Update UI elements with latest data"""
        # Update packet count and rate
        current_time = time.time()
        elapsed = current_time - self.last_update_time
        
        if elapsed > 0:
            rate = self.packets_since_last_update / elapsed
            self.rate_label.config(text=f"Rate: {rate:.1f} Hz")
            self.packets_since_last_update = 0
            self.last_update_time = current_time
        
        self.packet_count_label.config(text=f"Packets: {self.total_packets_received}")
        
        # Update recorded count if recording
        if self.is_recording:
            self.recorded_count_label.config(text=f"Recorded: {self.recorded_packets} packets")
        
        # Process text messages from serial monitor
        while not self.message_queue.empty():
            try:
                message = self.message_queue.get_nowait()
                self.monitor_text.config(state=tk.NORMAL)
                timestamp = datetime.now().strftime("%H:%M:%S")
                self.monitor_text.insert(tk.END, f"[{timestamp}] {message}\n")
                self.monitor_text.see(tk.END)
                
                # Keep only last 500 lines
                lines = self.monitor_text.get("1.0", tk.END).split('\n')
                if len(lines) > 501:
                    self.monitor_text.delete("1.0", f"{len(lines) - 500}.0")
                
                self.monitor_text.config(state=tk.DISABLED)
            except:
                pass
        
        # Process queued data: drain queue and keep only the latest value
        # At 50KHz, ~2500 packets arrive per 50ms update cycle
        # Draining is fast (just get_nowait in a loop) and ensures display shows latest
        latest_packet = None
        
        # Drain the queue to get the most recent value (fast operation)
        # We only need the latest for display, so we discard older packets
        while not self.data_queue.empty():
            try:
                latest_packet = self.data_queue.get_nowait()
            except:
                break
        
        # Update display with the latest value (if any)
        if latest_packet is not None:
            raw_value, rotor_angle, angular_velocity, sensor_magnitude, agc_value, timestamp = latest_packet
            self.latest_raw = raw_value
            self.latest_angle = rotor_angle
            self.latest_velocity = angular_velocity
            self.latest_magnitude = sensor_magnitude
            self.latest_agc = agc_value
            
            # Only display every Nth value in readout to prevent text widget lag
            self.display_counter += 1
            if self.display_counter >= 10:  # Display every 100th update (roughly every 5 seconds at 20Hz UI update rate)
                self.readout_text.config(state=tk.NORMAL)
                self.readout_text.insert(tk.END, 
                    f"Raw: {raw_value:8.4f}   |   Angle: {rotor_angle:8.3f} rad   |   Velocity: {angular_velocity:8.3f} rad/s   |   Mag: {sensor_magnitude:5d}   |   AGC: {agc_value:3d}\n")
                
                # Keep only last 50 lines
                lines = self.readout_text.get("1.0", tk.END).split('\n')
                if len(lines) > 51:
                    self.readout_text.delete("1.0", f"{len(lines) - 50}.0")
                
                self.readout_text.see(tk.END)
                self.readout_text.config(state=tk.DISABLED)
                self.display_counter = 0
        
        # Update latest values display
        if self.latest_raw is not None:
            self.raw_label.config(text=f"Raw: {self.latest_raw}")
        if self.latest_angle is not None:
            self.angle_label.config(text=f"Angle: {self.latest_angle:.6f} rad")
        if self.latest_velocity is not None:
            self.velocity_label.config(text=f"Velocity: {self.latest_velocity:.6f} rad/s")
        if self.latest_magnitude is not None:
            self.magnitude_label.config(text=f"Magnitude: {self.latest_magnitude}")
        if self.latest_agc is not None:
            self.agc_label.config(text=f"AGC: {self.latest_agc}")
        
        # Schedule next update
        self.root.after(50, self.update_ui)  # Update every 50ms


def main():
    root = tk.Tk()
    app = SensorNoiseRecorderUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
