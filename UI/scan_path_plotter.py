"""
Plotting script for expected vs actual motor angles.
Reads binary data from expected_vs_actual.bin and plots theta vs phi.
"""

import struct
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Data packet format (matches DataStream.hpp and motor_control_ui.py)
DATA_PACKET_SIZE = 19  # 1 (cmd) + 2 (distance) + 4 (angle1) + 4 (angle2) + 4 (target1) + 4 (target2)
DATA_PACKET_FORMAT = "<BHffff"  # little-endian: uint8, uint16, float, float, float, float
DATA_PACKET_STRUCT = struct.Struct(DATA_PACKET_FORMAT)


def read_binary_data(file_path):
    """
    Read binary data packets from file and return as list of tuples.
    
    Returns:
        list of tuples: (distance, angle1, angle2, target1, target2)
    """
    data = []
    
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Data file not found: {file_path}")
    
    with open(file_path, 'rb') as f:
        while True:
            packet_bytes = f.read(DATA_PACKET_SIZE)
            if len(packet_bytes) < DATA_PACKET_SIZE:
                break  # End of file or incomplete packet
            
            try:
                # Unpack packet: cmd, distance, angle1, angle2, target1, target2
                _, distance, angle1, angle2, target1, target2 = DATA_PACKET_STRUCT.unpack(packet_bytes)
                data.append((distance, angle1, angle2, target1, target2))
            except struct.error as e:
                print(f"Warning: Failed to unpack packet at position {f.tell() - DATA_PACKET_SIZE}: {e}")
                continue
    
    return data


def load_dataframe(file_path):
    """
    Load binary data into a pandas DataFrame.
    
    Args:
        file_path: Path to the binary data file
        
    Returns:
        pandas.DataFrame with columns: distance, theta_actual, phi_actual, theta_expected, phi_expected
    """
    data = read_binary_data(file_path)
    
    if not data:
        raise ValueError("No data found in file")
    
    # Create DataFrame
    # angle1 = theta actual, angle2 = phi actual
    # target1 = theta expected, target2 = phi expected
    df = pd.DataFrame(data, columns=['distance', 'theta_actual', 'phi_actual', 'theta_expected', 'phi_expected'])
    
    print(f"Loaded {len(df)} data packets")
    print(f"Data range:")
    print(f"  Theta actual: [{df['theta_actual'].min():.3f}, {df['theta_actual'].max():.3f}]")
    print(f"  Phi actual: [{df['phi_actual'].min():.3f}, {df['phi_actual'].max():.3f}]")
    print(f"  Theta expected: [{df['theta_expected'].min():.3f}, {df['theta_expected'].max():.3f}]")
    print(f"  Phi expected: [{df['phi_expected'].min():.3f}, {df['phi_expected'].max():.3f}]")
    
    return df


def plot_expected_vs_actual(df, save_path=None):
    """
    Plot expected vs actual motor angles as theta vs phi.
    
    Args:
        df: DataFrame with theta/phi actual and expected columns
        save_path: Optional path to save the plot
    """
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot expected path (targets)
    ax.plot(df['theta_expected'], df['phi_expected'], 
            label='Expected', 
            linewidth=1.5, 
            alpha=0.7,
            color='blue')
    
    # Plot actual path (measured angles)
    ax.plot(df['theta_actual'], df['phi_actual'], 
            label='Actual', 
            linewidth=1.5, 
            alpha=0.7,
            color='red')
    
    # Labels and formatting
    ax.set_xlabel('Theta (rad)', fontsize=12)
    ax.set_ylabel('Phi (rad)', fontsize=12)
    ax.set_title('Expected vs Actual Motor Angles', fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {save_path}")
    
    plt.show()


def main():
    """Main function to load data and create plot."""
    # Get the data file path (relative to this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_file = os.path.join(script_dir, 'data', 'raw_data.bin')
    
    try:
        # Load data into DataFrame
        df = load_dataframe(data_file)
        
        # Create plot
        plot_expected_vs_actual(df)
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Make sure you have recorded some data using the motor control UI.")
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
