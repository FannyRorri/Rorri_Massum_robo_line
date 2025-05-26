import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

import sys


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.odom_pose = None
        self.odom_velocity = None

        self.bridge = CvBridge()

        # intensities for each camera
        self.intensity_left = None
        self.intensity_mid = None
        self.intensity_right = None

        # bool values to see if the images for said camera have been received
        self.received_left = False
        self.received_mid = False
        self.received_right = False

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # subscribers to the cameras
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

    # callback to process the left camera input
    def left_callback(self, msg):
        self.get_logger().info("Left image received")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # ros img to openCV
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # convert to grayscale
        self.intensity_left = np.mean(gray)/255.0   # average pixel intensity
        self.received_left = True   # set bool to true since the values have been received

    # callback to process the middle camera input
    def middle_callback(self, msg):
        self.get_logger().info("Middle image received")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.intensity_mid = np.mean(gray) /255.0
        self.received_mid = True

    # callback to process the right camera input
    def right_callback(self, msg):
        self.get_logger().info("Right image received")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.intensity_right = np.mean(gray)/255.0
        self.received_right = True


    def update_callback(self):
        # wait to receive all the three cmaera images
        if not (self.received_left and self.received_mid and self.received_right):
            self.get_logger().info("Waiting for all camera images...")
            return
        # check if they are not null
        if (self.intensity_left is not None and  self.intensity_mid is not None and self.intensity_right is not None):
            self.get_logger().info(
                f"Intensities - Left: {self.intensity_left:.4f}, Mid: {self.intensity_mid:.4f}, Right: {self.intensity_right:.4f}"
            )

            v = 0.2
            cmd_vel = Twist()
            cmd_vel.linear.x = v
            cmd_vel.angular.z = 0.0

            left = self.intensity_left
            mid = self.intensity_mid
            right = self.intensity_right
            diff = right - left  # difference in intensities between the side cameras
            self.get_logger().info(f"Diff: {diff}")
            sf = 8.4 # steer factor for turning
            max_steer = 7.5
            if self.intensity_mid > 0.6:
                max_steer = 8.2

            # sharp turn when the difference is too big between the 2 cameras
            if diff > 0.05:
                sf = sf * 1.5
            else:
                sf = sf

            angular_z = sf * diff

            angular_z = max(min(angular_z, max_steer), -max_steer)

            # velocity
            base_v = 0.008
            max_v = 0.25
            max_diff = 0.15  # max allowed difference
            turn = min(abs(diff) / max_diff, 1.0) # normalize turn factor
            straight = 1.0 - turn
            v = base_v + (max_v - base_v) * straight 

            # reduce speed if you have lost the line in a camera
            no_line_thresh = 0.6
            if left > no_line_thresh and right > no_line_thresh:   # you don't see both lines
                v = 0.05  # slow down considerably to make the turn
            elif left > no_line_thresh or right > no_line_thresh:  # you don't see one line
                v = min(v, 0.08)

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
