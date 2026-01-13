#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /root/workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

LOG_DIR=/root/logs
mkdir -p "$LOG_DIR"

echo "=== Starting PX4 Flight Test (Headless with Chase Camera) ==="

# Start DDS Agent on DDS domain 0
echo "Starting MicroXRCE-DDS Agent on domain 0..."
MicroXRCEAgent udp4 -p 8888 -d 0 > "$LOG_DIR/dds_agent.log" 2>&1 &
DDS_PID=$!
echo "DDS Agent started (PID: $DDS_PID)"
sleep 2

# Copy chase camera world to PX4's worlds directory
echo "Setting up chase camera world file..."
mkdir -p /root/workspace/PX4-Autopilot/Tools/simulation/gz/worlds
cp /root/worlds/chase_camera.sdf /root/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/chase_camera.sdf
echo "Chase camera world file installed"

# Start PX4 SITL with Gazebo (headless with chase camera)
echo "Starting PX4 SITL with Gazebo (headless + chase camera)..."
cd /root/workspace/PX4-Autopilot

# Use standard x500 model with custom world containing chase camera
# The chase camera will follow the drone from a third-person perspective
HEADLESS=1 PX4_GZ_WORLD=chase_camera \
  make px4_sitl gz_x500 > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
echo "PX4 SITL started (PID: $PX4_PID)"

# Wait for PX4 and Gazebo to initialize
echo "Waiting for PX4 and Gazebo to initialize..."
sleep 15

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
    source /opt/ros/humble/setup.bash
    source /root/workspace/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=0
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

# Start Gazebo-ROS2 bridge for chase camera
echo "Starting Gazebo-ROS2 bridge for chase camera..."
ros2 run ros_gz_bridge parameter_bridge /chase_camera@sensor_msgs/msg/Image[gz.msgs.Image > "$LOG_DIR/gz_bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "Gazebo-ROS2 bridge started (PID: $BRIDGE_PID)"
sleep 3

# Start chase camera tracker (moves camera to follow drone)
echo "Starting chase camera tracker..."
python3 /root/scripts/chase_camera_tracker.py > "$LOG_DIR/chase_tracker.log" 2>&1 &
TRACKER_PID=$!
echo "Chase camera tracker started (PID: $TRACKER_PID)"
sleep 3

# Wait for chase camera ROS2 topic
echo "Waiting for chase camera ROS2 topic..."
timeout 30 bash -c '
    source /opt/ros/humble/setup.bash
    source /root/workspace/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=0
    while ! ros2 topic list | grep -q /chase_camera; do
        echo "Waiting for /chase_camera ROS2 topic..."
        sleep 2
    done
'

if [ $? -ne 0 ]; then
    echo "WARNING: Chase camera ROS2 topic not found"
    echo "Available ROS2 topics:"
    ros2 topic list | grep camera || true
    echo "Available Gazebo topics:"
    gz topic -l 2>/dev/null | grep camera || true
fi

echo "Chase camera ready!"

# Start camera recording from chase camera ROS2 topic
echo "Starting chase camera recording..."
python3 /root/scripts/record_camera.py "$LOG_DIR/flight_test.mp4" /chase_camera 1280 960 30 > "$LOG_DIR/camera_recorder.log" 2>&1 &
RECORDER_PID=$!
echo "Camera recorder started (PID: $RECORDER_PID)"
sleep 2

# Check recorder is running
if ! kill -0 $RECORDER_PID 2>/dev/null; then
    echo "WARNING: Camera recorder died, check logs"
    cat "$LOG_DIR/camera_recorder.log"
fi

# Run flight test
echo ""
echo "=== Running Flight Test ==="
python3 /root/scripts/flight_test.py

# Cleanup
echo ""
echo "=== Stopping Services ==="

# Stop camera recorder first (send SIGTERM for clean shutdown)
echo "Stopping camera recorder..."
kill -TERM $RECORDER_PID 2>/dev/null || true
sleep 2

# Stop chase camera tracker
echo "Stopping chase camera tracker..."
kill $TRACKER_PID 2>/dev/null || true

# Stop Gazebo-ROS2 bridge
echo "Stopping Gazebo-ROS2 bridge..."
kill $BRIDGE_PID 2>/dev/null || true

# Kill PX4 and DDS
kill $PX4_PID 2>/dev/null || true
kill $DDS_PID 2>/dev/null || true

echo ""
echo "=== Flight Test Complete ==="
echo "Logs available in: $LOG_DIR"
if [ -f "$LOG_DIR/flight_test.mp4" ]; then
    VIDEO_SIZE=$(du -h "$LOG_DIR/flight_test.mp4" | cut -f1)
    echo "Video saved: $LOG_DIR/flight_test.mp4 ($VIDEO_SIZE)"
else
    echo "WARNING: Video file not found!"
fi
