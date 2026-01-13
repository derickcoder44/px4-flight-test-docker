#!/usr/bin/env python3
"""
Chase camera tracker that follows the drone in Gazebo simulation.
Subscribes to drone pose and moves the chase camera to follow from behind/above.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import subprocess
import time


class ChaseCameraTracker(Node):
    def __init__(self):
        super().__init__('chase_camera_tracker')

        # Chase camera offset (relative to drone)
        self.offset_x = -5.0  # 5m behind
        self.offset_y = 0.0   # centered
        self.offset_z = 3.0   # 3m above
        self.look_ahead = 1.0 # look 1m ahead of drone

        self.drone_pose = None
        self.last_update_time = time.time()
        self.update_rate = 0.1  # Update camera every 100ms

        self.get_logger().info('Chase camera tracker started')

        # Create timer to update camera position
        self.timer = self.create_timer(self.update_rate, self.update_camera)

    def update_camera(self):
        """Update camera position using gz service to follow drone"""
        try:
            # Get x500 model pose from Gazebo
            result = subprocess.run(
                ['gz', 'model', '-m', 'x500', '-p'],
                capture_output=True,
                text=True,
                timeout=1.0
            )

            if result.returncode != 0:
                return

            # Parse pose (format: x y z roll pitch yaw)
            pose_str = result.stdout.strip()
            if not pose_str:
                return

            parts = pose_str.split()
            if len(parts) < 6:
                return

            drone_x = float(parts[0])
            drone_y = float(parts[1])
            drone_z = float(parts[2])
            drone_yaw = float(parts[5])

            # Calculate chase camera position (behind and above drone)
            cam_x = drone_x + self.offset_x * math.cos(drone_yaw) - self.offset_y * math.sin(drone_yaw)
            cam_y = drone_y + self.offset_x * math.sin(drone_yaw) + self.offset_y * math.cos(drone_yaw)
            cam_z = drone_z + self.offset_z

            # Calculate look-at point (slightly ahead of drone)
            look_x = drone_x + self.look_ahead * math.cos(drone_yaw)
            look_y = drone_y + self.look_ahead * math.sin(drone_yaw)
            look_z = drone_z

            # Calculate camera orientation to look at drone
            dx = look_x - cam_x
            dy = look_y - cam_y
            dz = look_z - cam_z

            cam_yaw = math.atan2(dy, dx)
            horizontal_dist = math.sqrt(dx*dx + dy*dy)
            cam_pitch = math.atan2(dz, horizontal_dist)
            cam_roll = 0.0

            # Set chase camera pose
            pose_cmd = f"{cam_x} {cam_y} {cam_z} {cam_roll} {cam_pitch} {cam_yaw}"
            subprocess.run(
                ['gz', 'model', '-m', 'chase_camera', '-p', pose_cmd],
                capture_output=True,
                timeout=0.5
            )

        except Exception as e:
            # Silently continue on errors (expected during startup)
            pass


def main(args=None):
    rclpy.init(args=args)
    tracker = ChaseCameraTracker()

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
