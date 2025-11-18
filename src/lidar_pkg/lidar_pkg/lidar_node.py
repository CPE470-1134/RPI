import sys
import serial
from lidar import LD19Packet, is_header
from plotters import CartesianPlotter


PORT = "/dev/ttyUSB0"
if sys.platform == "darwin":
    PORT = "/dev/cu.usbserial-0001"

BAUD = 230400


def main():
    print("Starting LiDAR Node...")
    
    # Initialize the plotter
    cartesian_plot_manager = CartesianPlotter()
    rotation_points = []
    last_angle = 0
    scan_count = 0
    
    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            print(f"Opened serial port: {ser.name}")
            
            while True:
                # Read until header is found
                b = ser.read()
                if is_header(b):
                    # Read remaining frame bytes
                    frame_data = bytes([b[0]]) + ser.read(LD19Packet.FRAME_SIZE - 1)
                    
                    try:
                        new_frame = LD19Packet(raw=frame_data)
                        
                        # Detect new rotation (360° scan complete)
                        if new_frame.start_angle < last_angle:
                            scan_count += 1
                        
                        # Update plot every 25 scans
                        if scan_count >= 25:
                            print(f"Updating plot - Scan count: {scan_count}")
                            cartesian_plot_manager.update(rotation_points)
                            scan_count = 0
                            rotation_points = []
                        
                        # Accumulate points
                        rotation_points.extend(new_frame.LDPoints)
                        last_angle = new_frame.start_angle
                        
                        # Display frame info
                        print(f"Frame - Start: {new_frame.start_angle:.1f}°, End: {new_frame.end_angle:.1f}°, "
                              f"Speed: {new_frame.speed / 64.0:.1f} RPM, Points: {len(rotation_points)}")
                        
                    except ValueError as e:
                        print(f"Error parsing frame: {e}")
                        continue
                        
    except KeyboardInterrupt:
        print("\nStopping LiDAR Node...")
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
