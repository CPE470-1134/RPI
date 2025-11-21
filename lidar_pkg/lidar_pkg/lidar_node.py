import sys
import serial
import numpy as np
from lidar import LD19Packet
from plotters import CartesianPlotter, visualize_opening
from opening_detector import PointCloud, get_opening


PORT = "/dev/ttyUSB0"
if sys.platform == "darwin":
    PORT = "/dev/cu.usbserial-0001"

BAUD = 230400

# Configuration
AGGREGATION_SCANS = 5  # Number of complete 360° rotations to aggregate
DETECT_OPENING = True  # Set to True to enable opening detection


def handle_rotation_completion(scan_count, rotation_points, point_cloud):
    scan_count += 1

    # Add points to cloud
    for pt in rotation_points:
        point_cloud.add_point(pt.distance, pt.intensity, pt.angle)

    print(f"Scan {scan_count} complete: {len(rotation_points)} points collected")

    # Check if we have enough scans
    if scan_count >= AGGREGATION_SCANS:
        print(f"\n✓ Aggregated {scan_count} scans with {point_cloud.size()} points")
        print("Detecting opening...")

        detect_and_visualize_opening(point_cloud, update_plot=False, save_visualization=True)
        return scan_count, True  # Signal to exit

    return scan_count, False


def detect_and_print_opening(point_cloud):
    result = get_opening(point_cloud)

    if result is not None:
        point1, point2, gap_size = result
        print(f"\n{'='*70}")
        print(f"✓ Opening Detected! (Discontinuity: {gap_size:.0f} mm)")
        print(f"{'='*70}")
        print(f"Edge Point 1: ({point1['x']:.1f}, {point1['y']:.1f})")
        print(f"Edge Point 2: ({point2['x']:.1f}, {point2['y']:.1f})")
        print(f"{'='*70}\n")
    else:
        print("ERROR: Could not detect opening")
        
    cartesian_plot = CartesianPlotter()
    cartesian_plot.update(point_cloud.points)
    
def detect_and_visualize_opening(point_cloud, update_plot=False, save_visualization=True):
    opening_result = get_opening(point_cloud)

    if opening_result is not None:
        p1, p2, gap_size = opening_result

        print("=" * 70)
        print(f"✓ Opening Detected! Discontinuity = {gap_size:.0f} mm")
        print("=" * 70)
        print(f"Edge Point 1: ({p1['x']:.1f}, {p1['y']:.1f})")
        print(f"Edge Point 2: ({p2['x']:.1f}, {p2['y']:.1f})")
        print("=" * 70 + "\n")

    else:
        print("ERROR: Could not detect opening")

    #if update_plot:
        #cartesian_plot = CartesianPlotter()
        #cartesian_plot.update(point_cloud.points)
    
    if save_visualization:
        # Delegate plotting
        visualize_opening(point_cloud, opening_result,filename="opening_detection.png")

def handle_visualization_update(scan_count, rotation_points, cartesian_plot_manager):
    if scan_count >= 25 and not DETECT_OPENING:
        print(f"Updating plot - Scan count: {scan_count}")
        cartesian_plot_manager.update(rotation_points)
        return 0, []

    return scan_count, rotation_points


def process_lidar_frame(new_frame, rotation_points, scan_count, last_angle, point_cloud, cartesian_plot_manager):
    should_exit = False
    
    if last_angle is None:
        last_angle = new_frame.start_angle
    #last_angle = new_frame.start_angle
    # Detect rotation completion
    if new_frame.start_angle < last_angle:
        
        print("Rotation complete.")
        scan_count, should_exit = handle_rotation_completion(scan_count, rotation_points, point_cloud)

        # Clear accumulated points for next rotation
        rotation_points = []    
        
        
    # Set last_angle to the end angle of the new frame
    last_angle = new_frame.end_angle
    
    # Update visualization (if not doing opening detection)
    #scan_count, rotation_points = handle_visualization_update(scan_count, rotation_points, cartesian_plot_manager)

    # Accumulate points
    #
    #print("Adding points from new frame:", len(new_frame.LDPoints))
    #print("Points:", [ (pt.angle, pt.distance) for pt in new_frame.LDPoints ])
    
    rotation_points.extend(new_frame.LDPoints)

    # Log frame info
    #print(f"Frame - Start: {new_frame.start_angle:.1f}°, End: {new_frame.end_angle:.1f}°, "
          #f"Speed: {new_frame.speed / 64.0:.1f} RPM, Points: {len(rotation_points)}")

    return rotation_points, scan_count, last_angle, should_exit


def parse_serial(ser):
    # Read until header is found
    b = ser.read()
    if not is_header(b):
        return None

    # Read remaining frame bytes
    frame_data = bytes([b[0]]) + ser.read(LD19Packet.FRAME_SIZE - 1)

    try:
        return LD19Packet(raw=frame_data)
    except ValueError as e:
        print(f"Error parsing frame: {e}")
        return None


def main():
    print("Starting LiDAR Node...")

    # Initialize the plotter
    cartesian_plot_manager = CartesianPlotter()

    # Initialize point cloud for opening detection
    point_cloud = PointCloud() if DETECT_OPENING else None

    rotation_points = []
    last_angle = 0
    scan_count = 0

    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            print(f"Opened serial port: {ser.name}")

            while True:
                new_frame = parse_serial(ser)

                if new_frame is None:
                    continue

                # Process Lidar_Frame
                # Accumulate points, check for rotation completion, and append to point cloud
                # detect opening if enough scans aggregated, print results
                rotation_points, scan_count, last_angle, should_exit = process_lidar_frame(
                    new_frame, rotation_points, scan_count, last_angle, point_cloud, cartesian_plot_manager
                )
                
                #print("Processed frame - New Size of rotation points:", len(rotation_points))
                #print("Scan Count:", scan_count)
                #print("--------------------------------")


                if should_exit:
    
                    return

    except KeyboardInterrupt:
        print("\nStopping LiDAR Node...")
        if not DETECT_OPENING:
            cartesian_plot_manager.close()
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Tabulate the frame data
def display_frame(new_frame):
    print("--------------------------------")
    print("Frame Data:")
    print("Hex Format: " + new_frame.raw.hex())
    print(f"Header: {0x54:02x}")
    print("Ver Length: " + str(new_frame.ver_len))
    print("Speed (RPM): " + str(new_frame.speed / 64.0))
    print("Start Angle (Degrees): " + str(new_frame.start_angle))
    print("End Angle (Degrees): " + str(new_frame.end_angle))
    print("Timestamp (ms): " + str(new_frame.timestamp))
    print()
    for i,pt in enumerate(new_frame.LDPoints):
        print(f"Point {i}: Distance (mm): {pt.distance}, Intensity: {pt.intensity}")
    print("CRC: " + str(new_frame.crc))
    print("--------------------------------")


def is_header(b : bytes):
    # Index First Byte
    return b[0] == LD19Packet.HEADER


if __name__ == '__main__':
    main()
