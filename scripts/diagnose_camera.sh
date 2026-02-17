#!/bin/bash
# Diagnostic script to check camera topic health

echo "=== Camera Diagnostic Tool ==="
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
source /root/workspace/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0

echo "1. Checking Gazebo topics:"
echo "-------------------------"
gz topic -l 2>/dev/null | grep -i camera || echo "  No camera topics found in Gazebo"
echo ""

echo "2. Checking Gazebo camera topic info:"
echo "-------------------------------------"
if gz topic -l 2>/dev/null | grep -q "/chase_camera"; then
    echo "  Topic /chase_camera exists in Gazebo"
    echo "  Checking message rate (5 second sample)..."
    timeout 5 gz topic -e -t /chase_camera 2>/dev/null | wc -l || echo "  Failed to read topic"
else
    echo "  Topic /chase_camera NOT found in Gazebo"
fi
echo ""

echo "3. Checking ROS2 topics:"
echo "------------------------"
ros2 topic list 2>/dev/null | grep -i camera || echo "  No camera topics found in ROS2"
echo ""

echo "4. Checking ROS2 /chase_camera topic info:"
echo "------------------------------------------"
if ros2 topic list 2>/dev/null | grep -q "/chase_camera"; then
    echo "  Topic /chase_camera exists in ROS2"
    ros2 topic info /chase_camera 2>/dev/null || echo "  Failed to get topic info"
    echo ""
    echo "  Checking message rate (5 second sample)..."
    timeout 5 ros2 topic hz /chase_camera 2>/dev/null || echo "  No messages received"
else
    echo "  Topic /chase_camera NOT found in ROS2"
fi
echo ""

echo "5. Checking Gazebo models:"
echo "-------------------------"
gz model -l 2>/dev/null || echo "  Failed to list models"
echo ""

echo "6. Checking if chase_camera model exists:"
echo "-----------------------------------------"
if gz model -l 2>/dev/null | grep -q "chase_camera"; then
    echo "  chase_camera model found"
    echo "  Model pose:"
    gz model -m chase_camera -p 2>/dev/null || echo "  Failed to get pose"
else
    echo "  chase_camera model NOT found"
fi
echo ""

echo "7. Checking Gazebo sensors:"
echo "--------------------------"
gz model -m chase_camera -l 2>/dev/null | grep -i sensor || echo "  No sensor info available"
echo ""

echo "=== Diagnostic Complete ==="
