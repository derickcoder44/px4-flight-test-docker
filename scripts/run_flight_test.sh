#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /root/workspace/ros2_ws/install/setup.bash

LOG_DIR=/root/logs
mkdir -p "$LOG_DIR"

echo "=== Starting PX4 Flight Test ==="

# Set up environment for GUI rendering in headless mode
export DISPLAY=:99
# Use software rendering for Mesa/OpenGL
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
# Force Gazebo to use GUI
export GZ_SIM_RENDER_ENGINE=ogre

# Source ROS2 environment globally for DDS agent
source /opt/ros/humble/setup.bash
source /root/workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# Start virtual display for video recording with GLX support
echo "Starting virtual display..."
Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset > "$LOG_DIR/xvfb.log" 2>&1 &
XVFB_PID=$!
sleep 3
echo "Virtual display started (PID: $XVFB_PID)"

# Verify display is ready
echo "Verifying display..."
DISPLAY=:99 xdpyinfo > /dev/null 2>&1 && echo "Display :99 is ready" || echo "WARNING: Display :99 not ready"

# Start DDS Agent (ROS2 environment already sourced above)
echo "Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/dds_agent.log" 2>&1 &
DDS_PID=$!
echo "DDS Agent started (PID: $DDS_PID)"
sleep 2

# Start PX4 SITL with Gazebo (GUI enabled for video recording)
echo "Starting PX4 SITL with Gazebo..."
cd /root/workspace/PX4-Autopilot
# Force Gazebo GUI to run (not headless)
export PX4_GZ_STANDALONE=0
export GZ_SIM_RENDER_ENGINE=ogre
make px4_sitl gz_x500 > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
echo "PX4 SITL started (PID: $PX4_PID)"

# Wait for PX4 to be ready
echo "Waiting for PX4 to initialize..."
sleep 10

# Wait for Gazebo window to appear and maximize it
echo "Waiting for Gazebo window..."
for i in {1..30}; do
    WINDOW_ID=$(DISPLAY=:99 xdotool search --onlyvisible --class gazebo 2>/dev/null | head -1)
    if [ -n "$WINDOW_ID" ]; then
        echo "Gazebo window found (ID: $WINDOW_ID)"
        DISPLAY=:99 xdotool windowsize $WINDOW_ID 1920 1080
        DISPLAY=:99 xdotool windowactivate $WINDOW_ID
        echo "Gazebo window maximized"
        break
    fi
    sleep 1
done

if [ -z "$WINDOW_ID" ]; then
    echo "WARNING: Gazebo window not found after 30s"
fi

# Start video recording now that window is ready
echo "Starting video recording..."
ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :99 \
    -c:v libx264 -preset ultrafast -pix_fmt yuv420p -crf 23 \
    "$LOG_DIR/flight_test.mp4" > "$LOG_DIR/ffmpeg.log" 2>&1 &
FFMPEG_PID=$!
echo "Video recording started (PID: $FFMPEG_PID)"
sleep 2

# Check if processes are still running
if ! kill -0 $DDS_PID 2>/dev/null; then
    echo "ERROR: DDS Agent died"
    cat "$LOG_DIR/dds_agent.log"
    exit 1
fi

if ! kill -0 $PX4_PID 2>/dev/null; then
    echo "ERROR: PX4 SITL died"
    tail -n 100 "$LOG_DIR/px4_sitl.log"
    exit 1
fi

# Wait for ROS topics to appear
echo "Waiting for ROS topics..."
timeout 60 bash -c '
    while ! ros2 topic list | grep -q /fmu/fmu/out/vehicle_status; do
        echo "Waiting for vehicle_status topic..."
        sleep 2
    done
'

if [ $? -ne 0 ]; then
    echo "ERROR: Timeout waiting for ROS topics"
    echo "=== Available topics ==="
    ros2 topic list || true
    echo "=== DDS Agent log ==="
    cat "$LOG_DIR/dds_agent.log"
    echo "=== PX4 SITL log (last 100 lines) ==="
    tail -n 100 "$LOG_DIR/px4_sitl.log"
    exit 1
fi

echo "ROS topics ready!"
echo "Available topics:"
ros2 topic list

# Run flight test
echo ""
echo "=== Running Flight Test ==="
python3 /root/scripts/flight_test.py

# Cleanup
echo ""
echo "=== Stopping Services ==="
kill $PX4_PID 2>/dev/null || true
kill $DDS_PID 2>/dev/null || true

# Stop video recording gracefully
echo "Stopping video recording..."
kill -INT $FFMPEG_PID 2>/dev/null || true
wait $FFMPEG_PID 2>/dev/null || true
sleep 2

# Stop virtual display
kill $XVFB_PID 2>/dev/null || true

echo ""
echo "=== Flight Test Complete ==="
echo "Logs available in: $LOG_DIR"
if [ -f "$LOG_DIR/flight_test.mp4" ]; then
    VIDEO_SIZE=$(du -h "$LOG_DIR/flight_test.mp4" | cut -f1)
    echo "Video saved: $LOG_DIR/flight_test.mp4 ($VIDEO_SIZE)"
else
    echo "WARNING: Video file not found!"
fi
