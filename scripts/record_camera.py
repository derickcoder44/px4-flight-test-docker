#!/usr/bin/env python3
"""
Record video from ROS2 camera topic to MP4 file.
Subscribes to sensor_msgs/Image and writes to video using OpenCV.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import signal

class CameraRecorder(Node):
    def __init__(self, output_file, topic_name, width=1280, height=960, fps=30):
        super().__init__('camera_recorder')

        self.output_file = output_file
        self.bridge = CvBridge()
        self.writer = None
        self.frame_count = 0
        self.width = width
        self.height = height
        self.fps = fps

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Waiting for images on topic: {topic_name}')
        self.get_logger().info(f'Output: {output_file} ({width}x{height} @ {fps} fps)')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Initialize video writer on first frame
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(
                    self.output_file,
                    fourcc,
                    self.fps,
                    (self.width, self.height)
                )
                self.get_logger().info('Video writer initialized')

            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))

            # Write frame
            self.writer.write(cv_image)
            self.frame_count += 1

            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Recorded {self.frame_count} frames')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def cleanup(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f'Video saved: {self.output_file} ({self.frame_count} frames)')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print('Usage: record_camera.py <output_file.mp4> <topic_name> [width] [height] [fps]')
        print('Example: record_camera.py output.mp4 /camera 1280 960 30')
        sys.exit(1)

    output_file = sys.argv[1]
    topic_name = sys.argv[2]
    width = int(sys.argv[3]) if len(sys.argv) > 3 else 1280
    height = int(sys.argv[4]) if len(sys.argv) > 4 else 960
    fps = int(sys.argv[5]) if len(sys.argv) > 5 else 30

    recorder = CameraRecorder(output_file, topic_name, width, height, fps)

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print('\nShutting down recorder...')
        recorder.cleanup()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.cleanup()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
