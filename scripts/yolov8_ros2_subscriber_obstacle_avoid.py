#!/usr/bin/env python3

import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()

from yolov8_msgs.msg import Yolov8Inference

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import math

from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Yolo_subscriber(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')

        self.image_width = 640
        self.ceased = False

        #code from yolov8 inference subscribing
        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)
        self.subscription 

        self.cnt = 0

        #####################################
        #code from obstacle avoid

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.linear_velocity = 0.0  # unit: m/s
        self.angular_velocity = 0.0  # unit: m/s
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning
        self.object_detected = False
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")



    def yolo_callback(self, data):

        self.object_detected = False

        for r in data.yolov8_inference:
            self.object_detected = True
            class_name = r.class_name
            top= r.top#x coord of top left point
            left= r.left#y coord of top left point
            bottom= r.bottom#x coord of bottom right point
            right= r.right#y coord of bottom right point
            self.left_bound = top
            self.right_bound = bottom
            yolo_subscriber.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")

            
            self.cnt += 1

        self.cnt = 0

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.init_scan_state = True

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True and self.object_detected is True:
            self.detect_obstacle()

    def odom_callback(self, msg):
        self.x_act = msg.pose.pose.position.x
        self.y_act = msg.pose.pose.position.y
        self.z_ori = msg.pose.pose.orientation.z
        # print ("Message received: ", self.x_act, self.y_act,self.z_ori)


    def detect_obstacle(self):
        twist = Twist()



        centre_of_picture = self.image_width/2
        centre_of_object = (self.right_bound+self.left_bound)//2

        forward_pixel_length = centre_of_picture/math.tan(31.1/180*math.pi)#half the fov angle
        tolerance = 2#push closer to object
        if centre_of_object>=centre_of_picture:#centre on right side of image
            #forward is 0 degrees, then follow astc increment of angle where right is 270 degrees
            angle = 360-round(math.atan(abs(centre_of_object-centre_of_picture)/forward_pixel_length)/math.pi*180)#obtain left most point angle
        elif centre_of_object<centre_of_picture:#centre on left side of image
            angle = round(math.atan(abs(centre_of_object-centre_of_picture)/forward_pixel_length)/math.pi*180)#obtain left most point angle
        
        
        if angle>=360 or angle <=0:
            angle = 0
        obstacle_distance = self.scan_ranges[angle]        


        safety_distance = 1  # unit: m
        self.get_logger().info(f"Potted plant detected at {obstacle_distance}m away at {angle}deg.")
        
        if obstacle_distance < safety_distance and not self.ceased:         
            self.ceased = True

            # Go to own current pose so new fake goal is added into navgiator object then we can easily cancel
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.x_act+2.0
            goal_pose.pose.position.y = self.y_act+0.63
            goal_pose.pose.orientation.z = self.z_ori


            navigator.goToPose(goal_pose)


            navigator.cancelTask()
            
            navigator.backup(backup_dist=1, backup_speed=0.26, time_allowance=10)#reverse




if __name__ == '__main__':
    rclpy.init(args=None)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    yolo_subscriber = Yolo_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = yolo_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
