"""
SimpleFOC Motor Control UI
A Tkinter-based interface for controlling gimbal motors via serial commands.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
from queue import Queue


class MotorControlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SimpleFOC Dual Motor Control")
        self.root.geometry("900x650")
        
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        self.message_queue = Queue()

        self.initial_angle = 0.0
        
        self.setup_ui()
        self.update_serial_monitor()
        
    def setup_ui(self):
        """Create the user interface"""
        # Configure grid weights for responsiveness
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        
        # ========== Connection Frame ==========
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        
        # COM Port selection
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        
        # Refresh ports button
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        
        # Baud rate (fixed at 115200 for this application)
        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, padx=5)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Label(conn_frame, textvariable=self.baud_var).grid(row=0, column=4, padx=5)
        
        # Connect/Disconnect button
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5)
        
        # Connection status
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)
        
        # ========== Control Frame ==========
        control_frame = ttk.LabelFrame(self.root, text="Motor Control - Angle Mode", padding=10)
        control_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        control_frame.grid_rowconfigure(1, weight=1)
        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=1)
        
        # Motor 1 Control
        motor1_frame = ttk.LabelFrame(control_frame, text="Motor 1", padding=10)
        motor1_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        ttk.Label(motor1_frame, text="Target Angle (rad):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.angle_entry1 = ttk.Entry(motor1_frame, width=15)
        self.angle_entry1.grid(row=0, column=1, padx=5, pady=5)
        self.angle_entry1.insert(0, "0.0")
        ttk.Button(motor1_frame, text="Set Angle", command=lambda: self.set_angle(1)).grid(row=0, column=2, padx=5, pady=5)
        
        # # Quick angle buttons for Motor 1
        # quick_frame1 = ttk.Frame(motor1_frame)
        # quick_frame1.grid(row=1, column=0, columnspan=3, padx=5, pady=5)
        # ttk.Label(quick_frame1, text="Quick:").pack(side=tk.LEFT)
        # for angle in [0, 0.5, 1.0, 1.5, 2.0, 2.5]:
        #     ttk.Button(quick_frame1, text=f"{angle}", width=6,
        #               command=lambda a=angle, m=1: self.quick_angle(a, m)).pack(side=tk.LEFT, padx=2)
        
        # Motor 2 Control
        motor2_frame = ttk.LabelFrame(control_frame, text="Motor 2", padding=10)
        motor2_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(motor2_frame, text="Target Angle (rad):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.angle_entry2 = ttk.Entry(motor2_frame, width=15)
        self.angle_entry2.grid(row=0, column=1, padx=5, pady=5)
        self.angle_entry2.insert(0, "0.0")
        ttk.Button(motor2_frame, text="Set Angle", command=lambda: self.set_angle(2)).grid(row=0, column=2, padx=5, pady=5)
        
        # # Quick angle buttons for Motor 2
        # quick_frame2 = ttk.Frame(motor2_frame)
        # quick_frame2.grid(row=1, column=0, columnspan=3, padx=5, pady=5)
        # ttk.Label(quick_frame2, text="Quick:").pack(side=tk.LEFT)
        # for angle in [0, 0.5, 1.0, 1.5, 2.0, 2.5]:
        #     ttk.Button(quick_frame2, text=f"{angle}", width=6,
        #               command=lambda a=angle, m=2: self.quick_angle(a, m)).pack(side=tk.LEFT, padx=2)
        
        # Serial Monitor
        monitor_frame = ttk.Frame(control_frame)
        monitor_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky="nsew")
        monitor_frame.grid_rowconfigure(1, weight=1)
        monitor_frame.grid_columnconfigure(0, weight=1)
        
        ttk.Label(monitor_frame, text="Serial Monitor", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w")
        
        self.serial_text = scrolledtext.ScrolledText(monitor_frame, height=15, width=80, 
                                                     bg="black", fg="lime", 
                                                     font=("Courier", 9))
        self.serial_text.grid(row=1, column=0, sticky="nsew")
        
        # Clear button for serial monitor
        ttk.Button(monitor_frame, text="Clear Monitor", 
                  command=self.clear_monitor).grid(row=2, column=0, pady=5)
        
        # ========== Additional Controls ==========
        extra_frame = ttk.Frame(self.root)
        extra_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        
        ttk.Button(extra_frame, text="Request Status (?)", 
                  command=self.request_status).pack(side=tk.LEFT, padx=5)
        ttk.Button(extra_frame, text="Motor Config (M)", 
                  command=self.motor_config).pack(side=tk.LEFT, padx=5)
        
    def refresh_ports(self):
        """Refresh the list of available COM ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Connect or disconnect from the serial port"""
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """Connect to the selected serial port"""
        port = self.port_combo.get()
        if not port:
            self.log_message("Error: No port selected\n")
            return
        
        try:
            baud = int(self.baud_var.get())
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.serial_port.dtr = True  # Enable DTR for Pico USB Serial
            self.is_connected = True
            self.running = True
            
            # Start reading thread
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
            # Update UI
            self.status_label.config(text=f"Connected to {port}", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.log_message(f"=== Connected to {port} at {baud} baud ===\n")
            
        except Exception as e:
            self.log_message(f"Error connecting: {str(e)}\n")
            self.is_connected = False
    
    def disconnect(self):
        """Disconnect from the serial port"""
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
        """Read data from serial port in a separate thread"""
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode('utf-8', errors='ignore')
                    self.message_queue.put(data)
            except Exception as e:
                if self.running:
                    self.message_queue.put(f"Read error: {str(e)}\n")
                break
            time.sleep(0.01)
    
    def update_serial_monitor(self):
        """Update the serial monitor with queued messages"""
        while not self.message_queue.empty():
            message = self.message_queue.get()
            self.serial_text.insert(tk.END, message)
            self.serial_text.see(tk.END)
        
        # Schedule next update
        self.root.after(50, self.update_serial_monitor)
    
    def log_message(self, message):
        """Add a message to the serial monitor"""
        self.serial_text.insert(tk.END, message)
        self.serial_text.see(tk.END)
    
    def clear_monitor(self):
        """Clear the serial monitor"""
        self.serial_text.delete(1.0, tk.END)
    
    def send_command(self, command):
        """Send a command to the motor via serial"""
        if not self.is_connected or not self.serial_port:
            self.log_message("Error: Not connected to serial port\n")
            return  
        
        try:
            self.serial_port.write(f"{command}\n".encode())
            self.log_message(f">>> Sent: {command}\n")
        except Exception as e:
            self.log_message(f"Error sending command: {str(e)}\n")
    
    def set_angle(self, motor_num):
        """Set the target angle for specified motor (X or Y)"""
        try:
            if motor_num == 1:
                angle = float(self.angle_entry1.get())
                self.send_command(f"X{angle}")
            elif motor_num == 2:
                angle = float(self.angle_entry2.get())
                self.send_command(f"Y{angle}")
        except ValueError:
            self.log_message(f"Error: Invalid angle value for Motor {motor_num}\n")
    
    
    def request_status(self):
        """Request motor status"""
        self.send_command("?")
    
    def motor_config(self):
        """Enter motor configuration mode"""
        self.send_command("M")


def main():
    root = tk.Tk()
    app = MotorControlUI(root)
    
    # Handle window close
    def on_closing():
        if app.is_connected:
            app.disconnect()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()

