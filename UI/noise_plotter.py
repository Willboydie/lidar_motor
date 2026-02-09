"""
Sensor Noise Plotter
Reads binary alpha_beta.bin file and plots raw + filtered angle, velocity estimate, magnitude, and AGC.
The binary file contains packets with:
  command token (0xAB) + float unfiltered_angle + float filtered_angle + float velocity + uint16 magnitude + uint16 agc
"""

import struct
import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path

# Data packet format: command token (0xAB) + float + float + float + uint16 + uint16 (little-endian)
DATA_PACKET_CMD = 0xAB
PACKET_SIZE = 17  # 1 (cmd) + 4 (float) + 4 (float) + 4 (float) + 2 (uint16) + 2 (uint16)
PACKET_FORMAT = "<BfffHH"  # little-endian: uint8, float, float, float, uint16, uint16
PACKET_STRUCT = struct.Struct(PACKET_FORMAT)


def read_data_file(filepath):
    """
    Read binary data file and extract all sensor fields.
    
    Returns:
        tuple: (raw_angles, filtered_angles, velocities, magnitudes, agc_values) as numpy arrays
    """
    raw_angles = []
    filtered_angles = []
    velocities = []
    magnitudes = []
    agc_values = []
    
    with open(filepath, 'rb') as f:
        buffer = bytearray()
        
        while True:
            chunk = f.read(4096)  # Read in chunks
            if not chunk:
                break
            
            buffer.extend(chunk)
            
            # Process complete packets
            while len(buffer) >= PACKET_SIZE:
                packet_bytes = buffer[:PACKET_SIZE]
                buffer = buffer[PACKET_SIZE:]
                
                try:
                    cmd, raw_angle, filtered_angle, velocity, magnitude, agc = PACKET_STRUCT.unpack(packet_bytes)
                    
                    if cmd == DATA_PACKET_CMD:
                        raw_angles.append(raw_angle)
                        filtered_angles.append(filtered_angle)
                        velocities.append(velocity)
                        magnitudes.append(magnitude)
                        agc_values.append(agc)
                    else:
                        # Invalid command token, skip this packet
                        print(f"Warning: Invalid command token 0x{cmd:02X}, skipping packet")
                        
                except struct.error as e:
                    print(f"Error unpacking packet: {e}")
                    # Try to resync by finding next command token
                    sync_idx = buffer.find(DATA_PACKET_CMD)
                    if sync_idx > 0:
                        buffer = buffer[sync_idx:]
                    else:
                        buffer.clear()
                    break
    
    return (np.array(raw_angles), np.array(filtered_angles),
            np.array(velocities), np.array(magnitudes), np.array(agc_values))


def plot_sensor_data(raw_angles, filtered_angles, velocities, magnitudes, agc_values,
                     sample_rate_hz=50000.0):
    """
    Plot sensor data in a 2x2 grid:
      Plot 1 (top-left):  Raw angle and filtered angle overlaid
      Plot 2 (top-right): Velocity estimate
      Plot 3 (bottom-left): Sensor magnitude
      Plot 4 (bottom-right): AGC value
    
    Args:
        raw_angles: Array of unfiltered rotor angles (radians)
        filtered_angles: Array of filtered rotor angles (radians)
        velocities: Array of estimated angular velocities (rad/s)
        magnitudes: Array of sensor magnitude readings (uint16)
        agc_values: Array of AGC readings (uint16)
        sample_rate_hz: Sample rate for time axis (default 50000 Hz)
    """
    if len(raw_angles) == 0:
        print("No data to plot")
        return
    
    # Create time axis
    dt = 1.0 / sample_rate_hz
    time_axis = np.arange(len(raw_angles)) * dt
    
    # Create figure with 2x2 grid of subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Sensor Noise Test Data', fontsize=14, fontweight='bold')
    
    # Plot 1: Raw angle and filtered angle overlaid
    axes[0, 0].plot(time_axis, raw_angles, 'b-', linewidth=0.5, alpha=0.5, label='Raw Angle')
    axes[0, 0].plot(time_axis, filtered_angles, 'r-', linewidth=0.5, alpha=0.7, label='Filtered Angle')
    axes[0, 0].set_title('Raw & Filtered Angle')
    axes[0, 0].set_ylabel('Angle (rad)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].legend(loc='upper right', fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)
    
    # Plot 2: Velocity estimate
    axes[0, 1].plot(time_axis, velocities, 'g-', linewidth=0.5, alpha=0.7)
    axes[0, 1].set_title('Velocity Estimate')
    axes[0, 1].set_ylabel('Angular Velocity (rad/s)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Sensor magnitude
    axes[1, 0].plot(time_axis, magnitudes, 'm-', linewidth=0.5, alpha=0.7)
    axes[1, 0].set_title('Sensor Magnitude')
    axes[1, 0].set_ylabel('Magnitude')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Plot 4: AGC value
    axes[1, 1].plot(time_axis, agc_values, 'c-', linewidth=0.5, alpha=0.7)
    axes[1, 1].set_title('AGC Value')
    axes[1, 1].set_ylabel('AGC')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].grid(True, alpha=0.3)
    
    # Adjust layout
    plt.tight_layout()
    
    # Print statistics
    print(f"\nData Statistics:")
    print(f"  Total samples: {len(raw_angles)}")
    print(f"  Duration: {time_axis[-1]:.3f} seconds")
    print(f"  Sample rate: {len(raw_angles) / time_axis[-1]:.1f} Hz")
    print(f"\nRaw Angle:")
    print(f"  Min: {np.min(raw_angles):.6f} rad, Max: {np.max(raw_angles):.6f} rad")
    print(f"  Mean: {np.mean(raw_angles):.6f} rad, Std: {np.std(raw_angles):.6f} rad")
    print(f"\nFiltered Angle:")
    print(f"  Min: {np.min(filtered_angles):.6f} rad, Max: {np.max(filtered_angles):.6f} rad")
    print(f"  Mean: {np.mean(filtered_angles):.6f} rad, Std: {np.std(filtered_angles):.6f} rad")
    print(f"\nVelocity Estimate:")
    print(f"  Min: {np.min(velocities):.6f} rad/s, Max: {np.max(velocities):.6f} rad/s")
    print(f"  Mean: {np.mean(velocities):.6f} rad/s, Std: {np.std(velocities):.6f} rad/s")
    print(f"\nSensor Magnitude:")
    print(f"  Min: {np.min(magnitudes)}, Max: {np.max(magnitudes)}")
    print(f"  Mean: {np.mean(magnitudes):.2f}, Std: {np.std(magnitudes):.2f}")
    print(f"\nAGC Value:")
    print(f"  Min: {np.min(agc_values)}, Max: {np.max(agc_values)}")
    print(f"  Mean: {np.mean(agc_values):.2f}, Std: {np.std(agc_values):.2f}")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Plot sensor noise test data from binary file')
    parser.add_argument('file', nargs='?', default='data/alpha_beta.bin',
                       help='Path to binary data file (default: data/alpha_beta.bin)')
    parser.add_argument('--rate', type=float, default=50000.0,
                       help='Sample rate in Hz (default: 50000.0)')
    
    args = parser.parse_args()
    
    # Resolve file path
    filepath = Path(args.file)
    if not filepath.is_absolute():
        # Relative to script directory
        script_dir = Path(__file__).parent.absolute()
        filepath = script_dir / args.file
    
    if not filepath.exists():
        print(f"Error: File not found: {filepath}")
        return
    
    print(f"Reading data from: {filepath}")
    
    try:
        raw_angles, filtered_angles, velocities, magnitudes, agc_values = read_data_file(filepath)
        
        if len(raw_angles) == 0:
            print("Error: No valid data packets found in file")
            return
        
        print(f"Successfully read {len(raw_angles)} packets")
        
        plot_sensor_data(raw_angles, filtered_angles, velocities, magnitudes, agc_values, args.rate)
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
