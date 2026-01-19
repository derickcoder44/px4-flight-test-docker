#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleControlMode

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile
        )

        self.control_mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/fmu/out/vehicle_control_mode',
            self.control_mode_callback,
            qos_profile
        )

        self.position_count = 0
        self.status_count = 0

    def position_callback(self, msg):
        self.position_count += 1
        if self.position_count % 10 == 0:  # Print every 1 second (10Hz)
            self.get_logger().info(
                f'Position: x={msg.x:.2f}m y={msg.y:.2f}m z={msg.z:.2f}m | '
                f'vx={msg.vx:.2f} vy={msg.vy:.2f} vz={msg.vz:.2f}'
            )

    def status_callback(self, msg):
        self.status_count += 1
        if self.status_count % 5 == 0:  # Print every 0.5 seconds
            nav_state_names = {
                0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION',
                4: 'AUTO_LOITER', 5: 'AUTO_RTL', 14: 'OFFBOARD', 17: 'AUTO_TAKEOFF'
            }
            arming_state_names = {1: 'INIT', 2: 'ARMED', 3: 'STANDBY', 4: 'DISARMED'}

            nav_state = nav_state_names.get(msg.nav_state, f'UNKNOWN({msg.nav_state})')
            arming_state = arming_state_names.get(msg.arming_state, f'UNKNOWN({msg.arming_state})')

            self.get_logger().info(
                f'Status: nav={nav_state} arming={arming_state} '
                f'failsafe={msg.failsafe}'
            )

    def control_mode_callback(self, msg):
        pass  # Don't spam with control mode updates

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticNode()

    print("=== Diagnostic Node Running ===")
    print("Monitoring vehicle position and status...")
    print("Press Ctrl+C to stop")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
