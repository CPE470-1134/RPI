import struct
import sys
import serial
import tabulate
from plotters import PolarPlotter, CartesianPlotter


CRC_TABLE =[
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
]
#HEADER = 0x54

PORT = "/dev/ttyUSB0"

# If Mac - Use /dev/cu.usbserial-0001 
if sys.platform == "darwin":
    PORT = "/dev/cu.usbserial-0001" 


BAUD = 230400
class LDPoint:
    def __init__(self, dist, intensity, angle):
        self.distance = dist
        self.intensity = intensity
        self.angle = angle
        

class LD19Packet:

    FRAME_SIZE = 47 # 47 bytes for frame data, 1 byte for CRC
    HEADER = 0x54 # 0x54 is the header byte
    POINT_PER_PACK = 12

    def __init__(self,raw: bytes):
        if len(raw) != self.FRAME_SIZE or not raw:
            raise ValueError("Invalid Frame Length (Expects Frame Size)")

        # raw bytes of Frame
        self.raw = raw
        #self.valid = self._valcrc()
        self.LDPoints = []
        self.speed = 0
        self.start_angle = 0.0
        self.end_angle = 0.0
        self.timestamp = 0
        self.crc = 0

        self._parse()
        
        if not self._valcrc():
            raise ValueError("Invalid CRC")

    # Validates CRC 
    def _valcrc(self):
        # CRC-8 per manual: start at 0, include all bytes except the CRC itself
        crc = 0
        for b in self.raw[0:-1]:
            crc = CRC_TABLE[(crc ^ b) & 0xff]

        frame_crc = self.raw[-1]  # last byte is CRC-8
        #print(f"CRC: {crc}, Frame CRC: {frame_crc}")
        return crc == frame_crc

    def _parse(self):

        # Byte layout offsets within a frame
        VER_LEN_OFFSET = 1
        SPEED_OFFSET = 2
        START_ANGLE_OFFSET = 4
        START_POINTS_OFFSET = 6
        END_ANGLE_OFFSET = 42
        TIMESTAMP_OFFSET = 44

        self.header = self.raw[0]
        self.ver_len = self.raw[VER_LEN_OFFSET]

        self.speed = struct.unpack_from("<H", self.raw, SPEED_OFFSET)[0]
        self.start_angle = struct.unpack_from("<H", self.raw, START_ANGLE_OFFSET)[0] / 100.0
        self.end_angle = struct.unpack_from("<H", self.raw, END_ANGLE_OFFSET)[0] / 100.0

        angle_diff = self.end_angle - self.start_angle
        if angle_diff < 0:
            angle_diff += 360.0

        point_count = 0
        
        next_point_offset = START_POINTS_OFFSET # speed + start_angle offset
        

        for _ in range(self.POINT_PER_PACK):
            #Unpack A Point (distance, intensity)
            # unpack 2 bytes dist, 1 byte intensity
            dist = struct.unpack_from("<H",self.raw,next_point_offset)[0]
            intensity = self.raw[next_point_offset + 2]
            
            t = point_count / (self.POINT_PER_PACK-1)
            angle = self.start_angle + angle_diff * t

            # Create and append new Point
            new_pt = LDPoint(dist, intensity, angle)
            self.LDPoints.append(new_pt)
            
            next_point_offset += 3
            point_count += 1

        self.timestamp = struct.unpack_from("<H",self.raw, TIMESTAMP_OFFSET)[0]

        self.crc = self.raw[-1] # last byte is CRC-8
        
def main():
    print("Hello from lab4!")

    # Read Serial Data from the USB with pyserial
    # on /dev/tStyUSB0

    #polar_plot_manager = PolarPlotter()
    cartesian_plot_manager = CartesianPlotter()
    rotation_points = []
    last_angle = 0
    scan_count = 0
    
    with serial.Serial(PORT,BAUD,timeout=1) as ser:
        print("Opened USB/Port Port:" + ser.name)

        try:
            while True:

                # Read Until Header is Found 
                # Once Found, Parse and Store LD Frame
                b = ser.read()
                if is_header(b):
                    #Parse
                    #print("Start of Frame Detected!")
                    
                    # Read FRAME_SIZE - 1 Bytes (Rest of Frame Besides Header )
                    frame_data = bytes([b[0]]) + ser.read(LD19Packet.FRAME_SIZE - 1)
                    new_frame = LD19Packet(raw=frame_data)

                   
                    # Check if a new rotation has started and we have enough points for a full scan
                    if new_frame.start_angle < last_angle:
                        scan_count += 1
                    
                    #print(scan_count)
                    if scan_count >= 25:
                        #print(f"Scan Count: {scan_count}")
                        #polar_plot_manager.update(rotation_points)
                        cartesian_plot_manager.update(rotation_points)
                        
                        #polar_plot_manager.save(f"./Polar/lidar_scan_polar_{scan_count}.png")
                       
                        scan_count = 0
                        rotation_points = []

                    rotation_points.extend(new_frame.LDPoints)
                    last_angle = new_frame.start_angle

                    display_frame(new_frame)
                    #cartesian_plot_manager.save(f"./Cartesian/lidar_scan_cartesian.png")

        except KeyboardInterrupt:
            print("Stopping...")
            #polar_plot_manager.close()
            cartesian_plot_manager.close()

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

if __name__ == "__main__":
    main()


