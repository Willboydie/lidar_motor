"""
Point Cloud Viewer for LIDAR Motor Data
Reads binary data from raw_data.bin, extracts angle pairs and distances,
converts to Cartesian coordinates, and displays as an Open3D point cloud.
"""

import struct
import os
import numpy as np

# Import Open3D
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Warning: Open3D library not found. Please install with: pip install open3d")

# Data packet format (matches DataStream.hpp and plotting.py)
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


def spherical_to_cartesian(distance, theta, phi, distance_unit='cm'):
    """
    Convert spherical coordinates to Cartesian coordinates.
    
    Coordinate system convention:
    - theta (angle1): azimuth angle (rotation around z-axis) in radians
    - phi (angle2): elevation angle (angle from horizontal plane) in radians
    - distance: radial distance from origin
    
    Cartesian coordinates:
    - x = distance * cos(phi) * cos(theta)
    - y = distance * cos(phi) * sin(theta)
    - z = distance * sin(phi)
    
    Args:
        distance: Radial distance (in distance_unit)
        theta: Azimuth angle in radians
        phi: Elevation angle in radians
        distance_unit: Unit of distance ('cm' or 'm')
    
    Returns:
        tuple: (x, y, z) in meters
    """
    # Convert distance to meters if needed
    if distance_unit == 'cm':
        r = distance / 100.0  # Convert cm to meters
    else:
        r = distance
    
    # Convert spherical to Cartesian
    x = r * np.cos(phi) * np.cos(theta)
    y = r * np.cos(phi) * np.sin(theta)
    z = r * np.sin(phi)
    
    return (x, y, z)


def create_point_cloud(data, use_actual_angles=True, distance_unit='cm'):
    """
    Create an Open3D point cloud from the data.
    
    Args:
        data: List of tuples (distance, angle1, angle2, target1, target2)
        use_actual_angles: If True, use actual angles (angle1, angle2), else use targets (target1, target2)
        distance_unit: Unit of distance ('cm' or 'm')
    
    Returns:
        open3d.geometry.PointCloud object
    """
    if not OPEN3D_AVAILABLE:
        raise RuntimeError("Open3D library is not available. Please install with: pip install open3d")
    
    # Extract angle pairs and distances
    points = []
    for packet in data:
        distance, angle1, angle2, target1, target2 = packet
        
        # Skip invalid distances (0 or very large values)
        if distance == 0 or distance > 50000:  # 500m max reasonable distance
            continue
        
        # Use actual angles or target angles
        if use_actual_angles:
            theta = angle1
            phi = angle2
        else:
            theta = target1
            phi = target2
        
        # Convert to Cartesian coordinates
        x, y, z = spherical_to_cartesian(distance, theta, phi, distance_unit)
        points.append([x, y, z])
    
    if len(points) == 0:
        raise ValueError("No valid points found in data")
    
    # Convert to numpy array
    points_array = np.array(points, dtype=np.float64)
    
    # Create Open3D point cloud
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_array)
    
    return cloud


def visualize_point_cloud(cloud):
    """
    Visualize the point cloud using Open3D visualizer.
    
    Args:
        cloud: open3d.geometry.PointCloud object
    """
    if not OPEN3D_AVAILABLE:
        raise RuntimeError("Open3D library is not available.")
    
    num_points = len(cloud.points)
    print(f"Visualizing point cloud with {num_points} points...")
    
    try:
        # Open3D visualization
        print("Point cloud viewer opened. Close the window to exit.")
        o3d.visualization.draw_geometries([cloud], 
                                         window_name="LIDAR Point Cloud",
                                         width=1024,
                                         height=768)
    except Exception as e:
        print(f"Visualization error: {e}")
        print("Attempting alternative visualization method...")
        # Fallback: save to file
        try:
            output_file = os.path.join(os.path.dirname(__file__), 'data', 'point_cloud.pcd')
            o3d.io.write_point_cloud(output_file, cloud)
            print(f"Point cloud saved to: {output_file}")
            print("You can view it using Open3D tools or other point cloud viewers.")
        except Exception as e2:
            print(f"Failed to save point cloud: {e2}")


def main():
    """Main function to load data and create point cloud visualization."""
    # Get the data file path (relative to this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_file = os.path.join(script_dir, 'data', 'raw_data.bin')
    
    try:
        # Read binary data
        print(f"Reading data from: {data_file}")
        data = read_binary_data(data_file)
        print(f"Loaded {len(data)} data packets")
        
        if len(data) == 0:
            print("Error: No data found in file")
            return
        
        # Extract statistics
        distances = [d[0] for d in data]
        angles1 = [d[1] for d in data]
        angles2 = [d[2] for d in data]
        
        print(f"\nData statistics:")
        print(f"  Distance range: [{min(distances)}, {max(distances)}] cm")
        print(f"  Angle1 (theta) range: [{min(angles1):.3f}, {max(angles1):.3f}] rad")
        print(f"  Angle2 (phi) range: [{min(angles2):.3f}, {max(angles2):.3f}] rad")
        
        # Create point cloud using actual angles
        print("\nCreating point cloud from actual angles...")
        cloud = create_point_cloud(data, use_actual_angles=True, distance_unit='cm')
        print(f"Point cloud created with {len(cloud.points)} points")
        
        # Visualize
        print("\nOpening point cloud viewer...")
        visualize_point_cloud(cloud)
        
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
