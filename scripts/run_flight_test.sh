#!/bin/bash
set -e

LOG_DIR=/root/logs
mkdir -p "$LOG_DIR"

echo "=== Starting PX4 Flight Test ==="

# Start DDS Agent
echo "Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/dds_agent.log" 2>&1 &
DDS_PID=$!
echo "DDS Agent started (PID: $DDS_PID)"
sleep 2

# Start PX4 SITL with Gazebo
echo "Starting PX4 SITL with Gazebo..."
cd /root/workspace/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500 > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
echo "PX4 SITL started (PID: $PX4_PID)"

# Wait for PX4 to be ready
echo "Waiting for PX4 to initialize..."
sleep 20

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

echo ""
echo "=== Flight Test Complete ==="
echo "Logs available in: $LOG_DIR"
