#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /root/workspace/ros2_ws/install/setup.bash

LOG_DIR=/root/logs
mkdir -p "$LOG_DIR"

echo "=== Starting PX4 Flight Test ==="

# Start Xvfb (virtual display)
echo "Starting virtual display..."
Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset > "$LOG_DIR/xvfb.log" 2>&1 &
XVFB_PID=$!
export DISPLAY=:99
echo "Xvfb started (PID: $XVFB_PID)"
sleep 1

# Start window manager
echo "Starting window manager..."
openbox --sm-disable &
WM_PID=$!
echo "Window manager started (PID: $WM_PID)"
sleep 1

# Configure for software rendering (needed for Xvfb)
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3
# Use OGRE (not OGRE2) which works better in virtual environments
export PX4_GZ_SIM_RENDER_ENGINE=ogre
# Camera follow offset (zoomed in closer to drone)
# Note: Uses Gazebo coordinate system (Z-up), not NED
export PX4_GZ_FOLLOW_OFFSET_X=-1.0  # 1m behind the drone
export PX4_GZ_FOLLOW_OFFSET_Y=0.0   # centered
export PX4_GZ_FOLLOW_OFFSET_Z=1.0   # 1m above the drone (Gazebo: positive Z is up)
# Unset HEADLESS to enable GUI (PX4 script checks if variable is unset, not if it's 0)
unset HEADLESS

# Start video recording
echo "Starting video recording..."
ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :99.0 \
    -vcodec libx264 -preset ultrafast -crf 23 \
    "$LOG_DIR/flight_test_recording.mp4" > "$LOG_DIR/ffmpeg.log" 2>&1 &
FFMPEG_PID=$!
echo "Video recording started (PID: $FFMPEG_PID)"
sleep 1

# Start DDS Agent
echo "Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/dds_agent.log" 2>&1 &
DDS_PID=$!
echo "DDS Agent started (PID: $DDS_PID)"
sleep 1

# Create custom PX4 startup script to enable logging
echo "Configuring PX4 logging..."
mkdir -p /root/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/microsd/etc
cat > /root/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/microsd/etc/extras.txt << 'EOF'
# Enable logging immediately
logger on
EOF

# Start PX4 SITL with Gazebo GUI (HEADLESS must be unset, not set to 0)
echo "Starting PX4 SITL with Gazebo GUI..."
cd /root/workspace/PX4-Autopilot
# Enable logging from start
export PX4_SIM_SPEED_FACTOR=1
make px4_sitl gz_x500 > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
echo "PX4 SITL started (PID: $PX4_PID)"

# Wait for Gazebo GUI to start (reduced delay)
sleep 5

# Wait for PX4 to be ready (reduced delay)
echo "Waiting for PX4 to initialize..."
sleep 8

# Check if processes are still running
if ! kill -0 $DDS_PID 2>/dev/null; then
    echo "ERROR: DDS Agent died"
    cat "$LOG_DIR/dds_agent.log"
    kill $PX4_PID 2>/dev/null || true
    kill -INT $FFMPEG_PID 2>/dev/null || true
    kill $WM_PID 2>/dev/null || true
    kill $XVFB_PID 2>/dev/null || true
    exit 1
fi

if ! kill -0 $PX4_PID 2>/dev/null; then
    echo "ERROR: PX4 SITL died"
    tail -n 100 "$LOG_DIR/px4_sitl.log"
    kill $DDS_PID 2>/dev/null || true
    kill -INT $FFMPEG_PID 2>/dev/null || true
    kill $WM_PID 2>/dev/null || true
    kill $XVFB_PID 2>/dev/null || true
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
    kill $PX4_PID 2>/dev/null || true
    kill $DDS_PID 2>/dev/null || true
    kill -INT $FFMPEG_PID 2>/dev/null || true
    kill $WM_PID 2>/dev/null || true
    kill $XVFB_PID 2>/dev/null || true
    exit 1
fi

echo "ROS topics ready!"

# Source ROS2 environment in main shell
source /opt/ros/humble/setup.bash
source /root/workspace/ros2_ws/install/setup.bash

echo "Available topics:"
ros2 topic list

# Setup camera to follow drone before flight test starts
echo ""
echo "Setting up camera to follow drone..."
for i in {1..3}; do
    if gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "x500_0"' > "$LOG_DIR/gz_follow.log" 2>&1; then
        echo "Camera follow enabled"
        break
    else
        echo "Retry $i: Camera follow failed"
        sleep 1
    fi
done

sleep 1

# Set camera offset for closer view
for i in {1..3}; do
    if gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -1.0, y: 0.0, z: 1.0' > "$LOG_DIR/gz_follow_offset.log" 2>&1; then
        echo "Camera offset set (closer view)"
        break
    else
        echo "Retry $i: Camera offset failed"
        sleep 1
    fi
done

# Run flight test
echo ""
echo "=== Running Flight Test ==="
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /root/workspace/ros2_ws/install/setup.bash
# Run with output logging
python3 /root/scripts/flight_test.py 2>&1 | tee "$LOG_DIR/flight_test.log"

# Cleanup
echo ""
echo "=== Stopping Services ==="

# Stop video recording first (send SIGINT for clean shutdown but don't wait)
echo "Stopping video recording..."
kill -INT $FFMPEG_PID 2>/dev/null || true

# Kill PX4, Gazebo, and DDS
kill $PX4_PID 2>/dev/null || true
kill $GZ_PID 2>/dev/null || true
kill $DDS_PID 2>/dev/null || true

# Stop video recording
echo "Stopping video recording..."
kill -INT $FFMPEG_PID 2>/dev/null || true
sleep 3  # Give ffmpeg time to finalize the video file

# Copy PX4 ulog files
echo "Copying PX4 ulog files..."
PX4_LOG_DIR="/root/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs/log"
if [ -d "$PX4_LOG_DIR" ]; then
    # Find and copy all .ulg files from the log directory (which is organized by date)
    find "$PX4_LOG_DIR" -name "*.ulg" -exec cp {} "$LOG_DIR/" \; 2>/dev/null
    ULOG_COUNT=$(ls -1 "$LOG_DIR"/*.ulg 2>/dev/null | wc -l)
    if [ "$ULOG_COUNT" -gt 0 ]; then
        echo "Copied $ULOG_COUNT ULog file(s)"
    else
        echo "No ULog files found in $PX4_LOG_DIR"
    fi
else
    echo "PX4 log directory not found: $PX4_LOG_DIR"
    echo "Checking alternative locations..."
    find /root/workspace/PX4-Autopilot/build -name "*.ulg" 2>/dev/null | head -5
fi

# Stop window manager and Xvfb
kill $WM_PID 2>/dev/null || true
kill $XVFB_PID 2>/dev/null || true

echo ""
echo "=== Flight Test Complete ==="
echo "Logs available in: $LOG_DIR"
echo "Video recording saved to: $LOG_DIR/flight_test_recording.mp4"
if ls "$LOG_DIR"/*.ulg 1> /dev/null 2>&1; then
    echo "ULog files:"
    ls -lh "$LOG_DIR"/*.ulg
else
    echo "No ULog files found"
    echo "Checking PX4 logs for logger messages..."
    grep -i "logger\|opened log file" "$LOG_DIR/px4_sitl.log" | tail -5 || echo "No logger messages found"
fi
