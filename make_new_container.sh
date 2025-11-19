#!/bin/bash

###############################################
# Create a new CPE470 Docker container
# Will Mount to Same ws as Previous Containers
# Usage: ./make_new_container.sh <container_name>
###############################################

if [ -z "$1" ]; then
    echo "❌ ERROR: No container name provided."
    echo "Usage: ./make_new_container.sh <container_name>"
    exit 1
fi

CONTAINER_NAME="$1"

echo "====================================="
echo " Creating container: $CONTAINER_NAME"
echo "====================================="

# --- Check if container exists ---
if docker ps -a --format '{{.Names}}' | grep -w "$CONTAINER_NAME" >/dev/null; then
    echo "⚠️ Container '$CONTAINER_NAME' already exists."

    # Stop container automatically
    echo "⏹ Stopping existing container..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true

    # Ask user if they want to overwrite
    read -p "Do you want to overwrite (remove + recreate) it? (y/N): " ANSWER
    case "$ANSWER" in
        [yY]|[yY][eE][sS])
            echo "🗑 Removing old container..."
            docker rm "$CONTAINER_NAME" >/dev/null 2>&1 || true
            ;;
        *)
            echo "❌ Aborting. Container '$CONTAINER_NAME' preserved."
            exit 1
            ;;
    esac
fi

echo "🚀 Launching new container..."

docker run -dit \
  --name "$CONTAINER_NAME" \
  --restart always \
  --env DISPLAY=:0 \
  --privileged \
  --net host \
  --pid host \
  -v /etc/group:/etc/group:ro \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/shadow:/etc/shadow:ro \
  -v /etc/sudoers.d:/etc/sudoers.d:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/kmjrjf/create3_ws:/root/create3_ws \
  --device /dev/video2:/dev/video2 \
  --device /dev/input/js0:/dev/input/js0 \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  --device /dev/ttyUSB1:/dev/ttyUSB1 \
  unrsaral/cpe470_670_create3

if [ $? -ne 0 ]; then
    echo "❌ ERROR: Failed to create container '$CONTAINER_NAME'."
    exit 1
fi

echo "====================================="
echo "✔ Container '$CONTAINER_NAME' created!"
echo "====================================="

echo ""
echo "NEXT STEPS:"
echo "1. Enter the container:"
echo "      docker exec -it $CONTAINER_NAME bash"
echo ""
echo "2. Run environment setup:"
echo "      chmod +x setup_new_container_env.sh"
echo "      ./setup_new_container_env.sh"
echo ""
echo "====================================="

