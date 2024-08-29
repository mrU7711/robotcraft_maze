#!/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNavigation:
    def __init__(self):  # Corrected __init__ method
        rospy.init_node('reactive_navigation', anonymous=True)
        
        # Initialize command velocity message
        self.cmd_vel = Twist()
        
        # Initialize sensor attributes
        self.front_sensor = None
        self.left_sensor = None
        self.right_sensor = None
        
        # Subscribe to IR sensors
        self.front_sub = rospy.Subscriber("base_scan_1", LaserScan, self.front_sensor_cb, queue_size=1)
        self.left_sub = rospy.Subscriber("base_scan_2", LaserScan, self.left_sensor_cb, queue_size=1)
        self.right_sub = rospy.Subscriber("base_scan_3", LaserScan, self.right_sensor_cb, queue_size=1)
        
        # Publisher for movement commands
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz
    
    def front_sensor_cb(self, msg):
        self.front_sensor = msg.ranges[0] if msg.ranges else None
    
    def left_sensor_cb(self, msg):
        self.left_sensor = msg.ranges[0] if msg.ranges else None
    
    def right_sensor_cb(self, msg):
        self.right_sensor = msg.ranges[0] if msg.ranges else None

    def calculate_command(self):
        if self.front_sensor is None or self.left_sensor is None or self.right_sensor is None:
            return
        
        front_distance = self.front_sensor
        left_distance = self.left_sensor
        right_distance = self.right_sensor

        # Adjusted wall-following logic with increased safe distances
        if front_distance < 0.24:  # Increased from 0.2 to 0.24 meters
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 3.0  # Turn left with increased speed
        elif right_distance < 0.12:  # Increased from 0.1 to 0.12 meters
            self.cmd_vel.linear.x = 0.3
            self.cmd_vel.angular.z = 1.5  # Slightly turn left with increased speed
        elif right_distance > 0.36:  # Increased from 0.3 to 0.36 meters
            self.cmd_vel.linear.x = 0.3
            self.cmd_vel.angular.z = -1.5  # Slightly turn right with increased speed
        else:  # Move forward if properly aligned
            self.cmd_vel.linear.x = 0.3
            self.cmd_vel.angular.z = 0.0

        # Publish the command
        self.cmd_vel_pub.publish(self.cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ReactiveNavigation()
        controller.run()
    except rospy.ROSInterruptException:
        pass
