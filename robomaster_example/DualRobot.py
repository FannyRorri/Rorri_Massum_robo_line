import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DualRobotController(Node):
    def __init__(self):
        super().__init__('dual_robot_controller')

        # Publishers
        self.cmd_pub_1 = self.create_publisher(Twist, '/rm1/cmd_vel', 10)
        self.cmd_pub_2 = self.create_publisher(Twist, '/rm2/cmd_vel', 10)

        # Subscribers
        self.odom_sub_1 = self.create_subscription(Odometry, '/rm1/odom', self.odom1_callback, 10)
        self.odom_sub_2 = self.create_subscription(Odometry, '/rm2/odom', self.odom2_callback, 10)

        # Vision subscribers
        self.bridge = CvBridge()
        self.middle_vision_r1 = None
        self.middle_vision_r2 = None
        self.sub_mid_vision_r1 = self.create_subscription(Image, '/rm1/middle_vision', self.middle_vision_r1_cb, 10)
        self.sub_mid_vision_r2 = self.create_subscription(Image, '/rm2/middle_vision', self.middle_vision_r2_cb, 10)

        # Speed tracking
        self.speed1 = 0.0
        self.speed2 = 0.0

        # Start time
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Control loop timer
        self.create_timer(0.1, self.control_loop)

    def odom1_callback(self, msg):
        v = msg.twist.twist.linear
        self.speed1 = (v.x ** 2 + v.y ** 2) ** 0.5

    def odom2_callback(self, msg):
        v = msg.twist.twist.linear
        self.speed2 = (v.x ** 2 + v.y ** 2) ** 0.5

    def middle_vision_r1_cb(self, msg):
        self.middle_vision_r1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def middle_vision_r2_cb(self, msg):
        self.middle_vision_r2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detect_line(self, image, color='red'):
        if image is None:
            return None  # No detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if color == 'red':
            mask1 = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255]))
            mask2 = cv2.inRange(hsv, np.array([160,100,100]), np.array([180,255,255]))
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == 'blue':
            mask = cv2.inRange(hsv, np.array([100,150,50]), np.array([140,255,255]))
        else:
            return None

        moments = cv2.moments(mask)
        area = moments['m00']
        if area > 500:  # Sufficient detection
            cx = int(moments['m10'] / area)
            width = mask.shape[1]
            center_offset = (cx - width // 2) / (width // 2)  # Normalize [-1, 1]
            return (center_offset, area)
        return None

    def control_loop(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # --- Robot 1 ---
        twist1 = Twist()
        detection_r1 = self.detect_line(self.middle_vision_r1, 'red')
        twist1.linear.x = 0.2
        if detection_r1:
            center_offset, area = detection_r1
            if center_offset < -0.3:
                twist1.angular.z = -0.5  # Steer right
            else:
                twist1.angular.z = 0.0
        else:
            twist1.angular.z = -0.5  # Lost red, search right
        self.cmd_pub_1.publish(twist1)

        # --- Robot 2 ---
        if current_time - self.start_time < 3:  # Small delay to avoid collision
            return
        twist2 = Twist()
        red_r2 = self.detect_line(self.middle_vision_r2, 'red')
        blue_r2 = self.detect_line(self.middle_vision_r2, 'blue')
        twist2.linear.x = 0.25

        if self.speed2 > self.speed1:
            # If Robot 2 is faster, prefer blue line
            if blue_r2:
                center_offset, area = blue_r2
                if abs(center_offset) > 0.3:
                    twist2.angular.z = 0.5  # Steer left
                else:
                    twist2.angular.z = 0.0  # Align with blue
            else:
                twist2.angular.z = 0.5  # Search for blue
        else:
            # Else, continue on red
            if red_r2:
                center_offset, area = red_r2
                if center_offset < -0.3:
                    twist2.angular.z = -0.5  # Steer right to reacquire red
                else:
                    twist2.angular.z = 0.0  # Align with red
            else:
                twist2.angular.z = -0.5  # Search for red
        self.cmd_pub_2.publish(twist2)

def main(args=None):
    rclpy.init(args=args)
    node = DualRobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

