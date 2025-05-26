import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import sys


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.odom_pose = None
        self.odom_velocity = None

        # camera sensors
        self.left_v = None
        self.mid_v = None
        self.right_v = None

        self.bridge = CvBridge()

        self.intensity_left = None
        self.intensity_mid = None
        self.intensity_right = None

        self.received_left = False
        self.received_mid = False
        self.received_right = False

        # steering
        self.prev_diff = 0.0
        self.alpha = 0.7
        self.smoothed_diff = 0.0

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.left_v = self.create_subscription(Image, '/left_vision', self.left_callback, 10)
        self.mid_v = self.create_subscription(Image, '/middle_vision', self.middle_callback, 10)
        self.right_v = self.create_subscription(Image, '/right_vision', self.right_callback, 10)


    def start(self):
        self.timer = self.create_timer(1 / 60, self.update_callback)

    def stop(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
            throttle_duration_sec=0.5  # Throttle logging frequency to max 2Hz
        )

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw  # theta orientation
        )

        return pose2

    def left_callback(self, msg):
        self.get_logger().info("Left image received")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.intensity_left = np.mean(gray)/255.0
        self.received_left = True

    def middle_callback(self, msg):
        self.get_logger().info("Middle image received")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.intensity_mid = np.mean(gray) /255.0
        self.received_mid = True

    def right_callback(self, msg):
        self.get_logger().info("Right image received")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.intensity_right = np.mean(gray)/255.0
        self.received_right = True


    def update_callback(self):
        if not (self.received_left and self.received_mid and self.received_right):
            self.get_logger().info("Waiting for all camera images...")
            return

        if (
                self.intensity_left is not None and
                self.intensity_mid is not None and
                self.intensity_right is not None
        ):
            self.get_logger().info(
                f"Intensities - Left: {self.intensity_left:.4f}, Mid: {self.intensity_mid:.4f}, Right: {self.intensity_right:.4f}"
            )

            # for straight line paths
            # threshold = 0.6
            #
            # if self.intensity_mid > threshold:
            #     self.get_logger().info("Line finished: stopping")
            #     self.stop()
            #     return

            v = 0.2
            cmd_vel = Twist()
            cmd_vel.linear.x = v
            cmd_vel.angular.z = 0.0

            left = self.intensity_left
            right = self.intensity_right
            diff = right - left
            sf = 2.5
            angular_z = sf * diff

            max_steer = 1.0
            angular_z = max(min(angular_z, max_steer), -max_steer)

            base_v = 0.2
            min_v = 0.04
            max_diff = 0.5
            t = min(abs(diff) / max_diff, 1.0)
            v = base_v - (base_v - min_v) * t

            no_line_thresh = 0.6
            if left > no_line_thresh or right > no_line_thresh:
                v = min(v, 0.08)

            # Final command
            cmd_vel = Twist()
            cmd_vel.linear.x = v
            cmd_vel.angular.z = angular_z

            self.get_logger().info(
                f"Steering with diff={diff:.4f}, angular.z={angular_z:.4f}, v={v:.2f}, L={left:.4f}, R={right:.4f}"
            )

            self.vel_publisher.publish(cmd_vel)


def main():
    rclpy.init(args=sys.argv)

    node = ControllerNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop()


if __name__ == '__main__':
    main()
