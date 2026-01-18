import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class SmartStreetCleaner(Node):
    def __init__(self):
        super().__init__('smart_cleaner_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.obstacle_dist = 10.0
        self.trash_detected = False
        self.get_logger().info('--- Smart Cleaner logic is Online ---')

    def laser_callback(self, msg):
        # קריאת מרחק מהלייזר (גזרה קדמית)
        front_ranges = msg.ranges[0:20] + msg.ranges[-20:]
        self.obstacle_dist = min([r for r in front_ranges if r > 0.1])

    def camera_callback(self, msg):
        # זיהוי אובייקטים באמצעות OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # זיהוי צבע ירוק (העמודים/לכלוך בסימולציה)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        self.trash_detected = np.sum(mask) > 50000 # סף זיהוי

    def control_loop(self):
        move = Twist()
        
        if self.trash_detected:
            # מצב ניקוי: סיבוב במקום כדי "לאסוף" את הלכלוך
            self.get_logger().info('!!! TRASH DETECTED - Cleaning in progress !!!')
            move.linear.x = 0.0
            move.angular.z = 1.0 
        elif self.obstacle_dist < 0.6:
            # הימנעות ממכשול (קיר/רהיט)
            move.linear.x = 0.0
            move.angular.z = 0.5
        else:
            # נסיעה רגילה ברחוב
            move.linear.x = 0.2
            move.angular.z = 0.0
            
        self.publisher_.publish(move)

def main(args=None):
    rclpy.init(args=args)
    node = SmartStreetCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()