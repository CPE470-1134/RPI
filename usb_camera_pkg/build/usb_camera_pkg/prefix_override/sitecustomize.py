import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/RPI/usb_camera_pkg/install/usb_camera_pkg'
