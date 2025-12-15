i#!/bin/bash
set -e

echo "====================================="
echo "     ROS2 ENVIRONMENT SETUP SCRIPT    "
echo "====================================="

###############################################################################
# TASK A — MINIMAL PYTHON + ROS2 SETUP
###############################################################################
echo ""
echo "====================================="
echo " TASK A: Minimal ROS2 + Python Setup "
echo "====================================="

echo "[A1] Removing old ROS2 sources and keyrings..."
sudo rm -f /etc/apt/sources.list.d/*ros*

echo "[A2] Installing new ROS2 Humble key..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg

echo "[A3] Adding ROS2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "[A4] Updating and installing minimal Python tools..."
sudo apt update -y || true
sudo apt install -y python3-pip python3-setuptools python3-wheel python3-dev python3-venv

echo "[A5] Installing Python requirements..."
REQ="/root/create3_ws/src/RPI/requirements.txt"
if [ -f "$REQ" ]; then
    pip3 install --upgrade pip
    pip3 install -r "$REQ"
else
    echo "⚠️ WARNING: $REQ not found."
fi


echo "[A6] Updating .bashrc (ROS2 + Domain ID 12)..."
sed -i '/ROS_DOMAIN_ID/d' ~/.bashrc
sed -i '/RMW_IMPLEMENTATION/d' ~/.bashrc
sed -i '/create3_ws\/install\/setup.bash/d' ~/.bashrc

cat << 'EOF' >> ~/.bashrc

###############################################
# ROS2 Humble Environment
###############################################
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /root/create3_ws/install/setup.bash ] && source /root/create3_ws/install/setup.bash

# DDS + Domain ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=12

EOF

###############################################################################
# TASK B — GUI/X11 HOST ACCESS (DISPLAY=:0)
###############################################################################
echo ""
echo "====================================="
echo " TASK B: Host GUI (X11) Setup        "
echo "====================================="

echo "[B1] Installing X11 + GUI libraries..."
sudo apt install -y \
    xauth x11-apps \
    libx11-6 libx11-xcb1 libxcb1 libxcb-xfixes0 libxcb-xinerama0

echo "[B2] Adding DISPLAY and QT fixes to .bashrc..."
sed -i '/export DISPLAY/d' ~/.bashrc
sed -i '/QT_X11_NO_MITSHM/d' ~/.bashrc

cat << 'EOF' >> ~/.bashrc

###############################################
# GUI / X11 Support
###############################################
export DISPLAY=:0
export QT_X11_NO_MITSHM=1

EOF

###############################################################################
# HEALTH CHECK — Python, ROS2, Serial, Topics
###############################################################################
echo ""
echo "====================================="
echo "        RUNNING HEALTH CHECK         "
echo "====================================="

python3 - <<EOF
import os, serial.tools.list_ports, subprocess, sys

print("\n=========== PYTHON ENV ===========")
print("Python:", sys.executable)
print("\nInstalled Python Packages:")
subprocess.run(["pip3", "list"])

print("\n=========== SERIAL PORTS ===========")
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(" -", p.device)

print("\n=========== ROS2 TOPICS ===========")
try:
    subprocess.run(["ros2", "topic", "list"])
except:
    print("❌ ros2 not available")

print("\n=========== ENV VARS ===========")
print("DISPLAY:", os.environ.get("DISPLAY"))
print("RMW:", os.environ.get("RMW_IMPLEMENTATION"))
print("DOMAIN:", os.environ.get("ROS_DOMAIN_ID"))
EOF

echo ""
echo "====================================="
echo "  ENVIRONMENT SETUP COMPLETE ✔️      "
echo "====================================="

