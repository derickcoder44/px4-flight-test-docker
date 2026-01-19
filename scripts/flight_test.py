#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode, VehicleStatus
import time

class FlightTestNode(Node):
    def __init__(self):
        super().__init__('flight_test_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/fmu/in/offboard_control_mode',
            qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/fmu/in/trajectory_setpoint',
            qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/fmu/in/vehicle_command',
            qos_profile
        )

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        # State
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        # Flight parameters
        self.takeoff_height = -5.0  # meters (NED frame, negative is up)
        self.hover_duration = 5.0  # seconds

        # Waypoints (NED frame: North, East, Down)
        # Smaller 10m x 10m pattern for better visibility
        self.waypoints = [
            (10.0, 0.0, self.takeoff_height),   # 10m North
            (10.0, 10.0, self.takeoff_height),  # 10m North, 10m East
            (0.0, 10.0, self.takeoff_height),   # 10m East only
            (0.0, 0.0, self.takeoff_height),    # Return to home position
        ]

        self.get_logger().info('Flight test node initialized')

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.get_logger().info('Sending arm command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # Keep publishing setpoints while waiting for arm to complete
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            time.sleep(0.1)

    def disarm(self):
        self.get_logger().info('Sending disarm command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        time.sleep(1)

    def engage_offboard_mode(self):
        self.get_logger().info('Engaging offboard mode')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        # Keep publishing setpoints while waiting for mode change
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            time.sleep(0.1)

    def land(self):
        self.get_logger().info('Sending land command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        time.sleep(1)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def fly_to_waypoint(self, x, y, z, duration=3.0):
        """Fly to a specific waypoint and hold for duration"""
        self.get_logger().info(f'Flying to waypoint: ({x:.1f}m N, {y:.1f}m E, {-z:.1f}m altitude)')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x, y, z)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)
        self.get_logger().info(f'Waypoint reached and held for {duration}s')

    def run_flight_test(self):
        self.get_logger().info('Starting flight test sequence')

        # Wait for vehicle status
        self.get_logger().info('Waiting for vehicle status...')
        while self.vehicle_status.timestamp == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Vehicle status received')

        # Send setpoints before engaging offboard mode (PX4 requires streaming setpoints)
        self.get_logger().info('Streaming initial setpoints (5 seconds)')
        for i in range(50):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            time.sleep(0.1)

        # Engage offboard mode first, then arm
        self.engage_offboard_mode()

        # Continue streaming setpoints after engaging offboard mode
        for i in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            time.sleep(0.1)

        self.arm()

        # Takeoff
        self.get_logger().info(f'Taking off to {-self.takeoff_height}m')
        start_time = time.time()
        while time.time() - start_time < 10.0:  # 10 seconds to reach altitude
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)

        self.get_logger().info('Takeoff complete')

        # Hover at takeoff position
        self.get_logger().info(f'Hovering at takeoff position for {self.hover_duration}s')
        start_time = time.time()
        while time.time() - start_time < self.hover_duration:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)

        # Fly to waypoints
        for i, (x, y, z) in enumerate(self.waypoints, 1):
            self.get_logger().info(f'=== Waypoint {i}/{len(self.waypoints)} ===')
            self.fly_to_waypoint(x, y, z, duration=3.0)

        # Hover at home before landing
        self.get_logger().info('Arrived at home position, hovering before landing')
        start_time = time.time()
        while time.time() - start_time < self.hover_duration:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)

        # Land
        self.get_logger().info('Landing')
        self.land()

        # Wait for landing
        time.sleep(15)

        self.get_logger().info('Flight test complete')

def main(args=None):
    rclpy.init(args=args)
    node = FlightTestNode()

    try:
        node.run_flight_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.disarm()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
