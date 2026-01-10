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

# Start DDS Agent on DDS domain 0 (ROS2 environment already sourced above)
echo "Starting MicroXRCE-DDS Agent on domain 0..."
MicroXRCEAgent udp4 -p 8888 -d 0 > "$LOG_DIR/dds_agent.log" 2>&1 &
DDS_PID=$!
echo "DDS Agent started (PID: $DDS_PID)"
sleep 2

# Build PX4 if needed (silent build check)
cd /root/workspace/PX4-Autopilot
if [ ! -f "./build/px4_sitl_default/bin/px4" ]; then
    echo "Building PX4 SITL..."
    make px4_sitl_default > "$LOG_DIR/px4_build.log" 2>&1
fi

# Start Gazebo with GUI (force Qt to use X11 display)
echo "Starting Gazebo with GUI..."
echo "Display check: DISPLAY=$DISPLAY"

# Force Qt to use X11 backend (not offscreen or wayland)
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH=/root/workspace/PX4-Autopilot/Tools/simulation/gz/models:/root/workspace/PX4-Autopilot/Tools/simulation/gz/worlds

# Start gz-sim WITHOUT -s flag (server+gui mode), WITH -r to run immediately
gz sim -v4 -r /root/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf > "$LOG_DIR/gz_sim.log" 2>&1 &
GZ_PID=$!
echo "Gazebo started (PID: $GZ_PID) with QT_QPA_PLATFORM=xcb"
sleep 10

# Now start PX4 SITL (it will connect to the running Gazebo instance)
echo "Starting PX4 SITL..."
cd /root/workspace/PX4-Autopilot
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_GZ_MODEL=x500 PX4_SYS_AUTOSTART=4001 \
./build/px4_sitl_default/bin/px4 -i 1 -d > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
echo "PX4 SITL started (PID: $PX4_PID)"

# Wait for PX4 to be ready
echo "Waiting for PX4 to initialize..."
sleep 10

# Wait for Gazebo window to appear and maximize it
echo "Waiting for Gazebo window..."
echo "Listing all windows..."
DISPLAY=:99 xdotool search --name ".*" 2>/dev/null || echo "No windows found"

# Disable exit-on-error for window detection
set +e

for i in {1..30}; do
    # Try multiple patterns - gz-sim, Gazebo, Scene, or any window
    WINDOW_ID=$(DISPLAY=:99 xdotool search --name "Gazebo\|Scene\|gz" 2>/dev/null | head -1)

    if [ -z "$WINDOW_ID" ]; then
        # Try by class name
        WINDOW_ID=$(DISPLAY=:99 xdotool search --class "gz" 2>/dev/null | head -1)
    fi

    if [ -z "$WINDOW_ID" ]; then
        # Just get any window
        WINDOW_ID=$(DISPLAY=:99 xdotool search --name ".*" 2>/dev/null | head -1)
    fi

    if [ -n "$WINDOW_ID" ]; then
        echo "Window found (ID: $WINDOW_ID)"

        # Get window info (don't fail if these commands error)
        WINDOW_NAME=$(DISPLAY=:99 xdotool getwindowname $WINDOW_ID 2>/dev/null)
        if [ $? -eq 0 ] && [ -n "$WINDOW_NAME" ]; then
            echo "  Window name: $WINDOW_NAME"
        else
            echo "  Window name: (unknown)"
        fi

        WINDOW_CLASS=$(DISPLAY=:99 xdotool getwindowclassname $WINDOW_ID 2>/dev/null)
        if [ $? -eq 0 ] && [ -n "$WINDOW_CLASS" ]; then
            echo "  Window class: $WINDOW_CLASS"
        else
            echo "  Window class: (unknown)"
        fi

        # Resize and activate (ignore errors)
        DISPLAY=:99 xdotool windowsize $WINDOW_ID 1920 1080 2>/dev/null || true
        DISPLAY=:99 xdotool windowactivate $WINDOW_ID 2>/dev/null || true
        echo "Window resized to 1920x1080"
        break
    fi

    echo "  Attempt $i: No window found yet..."
    sleep 2
done

if [ -z "$WINDOW_ID" ]; then
    echo "WARNING: No window found after 60s"
    echo "Listing all X11 clients:"
    DISPLAY=:99 xlsclients 2>/dev/null || echo "xlsclients failed"
fi

# Re-enable exit-on-error
set -e

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

# Stop video recording first (send SIGINT for clean shutdown but don't wait)
echo "Stopping video recording..."
kill -INT $FFMPEG_PID 2>/dev/null || true

# Kill PX4, Gazebo, and DDS
kill $PX4_PID 2>/dev/null || true
kill $GZ_PID 2>/dev/null || true
kill $DDS_PID 2>/dev/null || true

# Give ffmpeg 3 seconds to finalize, then force kill if needed
sleep 3
if kill -0 $FFMPEG_PID 2>/dev/null; then
    echo "Force killing ffmpeg..."
    kill -9 $FFMPEG_PID 2>/dev/null || true
fi

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
