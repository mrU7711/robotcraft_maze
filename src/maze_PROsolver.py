#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import tf

class PathFollower:
    def __init__(self, start_x, start_y, path):
        rospy.init_node('path_follower', anonymous=True)
        
        # Publisher for movement commands
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Subscriber for the robot's odometry and laser scans
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laser_sub_0 = rospy.Subscriber("/base_scan_0", LaserScan, self.laser_callback)
        self.laser_sub_1 = rospy.Subscriber("/base_scan_1", LaserScan, self.laser_callback)
        self.laser_sub_2 = rospy.Subscriber("/base_scan_2", LaserScan, self.laser_callback)
        self.laser_sub_3 = rospy.Subscriber("/base_scan_3", LaserScan, self.laser_callback)
        
        self.path = path
        self.start_position = (start_x, start_y)
        self.current_position = self.start_position
        self.current_orientation = None
        self.front_distance = float('inf')
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_orientation = yaw

    def laser_callback(self, msg):
        # Process laser scan data
        min_distance = min(msg.ranges)
        if min_distance < self.front_distance:
            self.front_distance = min_distance

    def move_to_waypoint(self, target_x, target_y):
        while not rospy.is_shutdown():
            if self.current_position is None or self.current_orientation is None:
                continue

            # Calculate the distance and angle to the target
            dx = target_x - self.current_position[0]
            dy = target_y - self.current_position[1]
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.current_orientation)

            # Set movement commands
            cmd = Twist()

            # Obstacle avoidance
            if self.front_distance < 0.5:
                cmd.linear.x = -0.1  # Move backward slightly to avoid the obstacle
                cmd.angular.z = 0.5  # Turn to avoid the obstacle
            else:
                if distance > 0.1:  # If not at the waypoint
                    cmd.linear.x = 0.2  # Move forward
                else:
                    cmd.linear.x = 0.0  # Stop forward motion

                if abs(angle_diff) > 0.1:  # If not facing the target
                    cmd.angular.z = 0.5 * angle_diff  # Rotate towards the waypoint
                else:
                    cmd.angular.z = 0.0  # Stop rotation

            self.cmd_vel_pub.publish(cmd)

            if distance < 0.1 and abs(angle_diff) < 0.1:  # Reached the waypoint
                break

            self.rate.sleep()

    def normalize_angle(self, angle):
        """ Normalize an angle to [-pi, pi] """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def follow_path(self):
        rospy.loginfo(f"Starting at position: {self.start_position}")
        for waypoint in self.path:
            rospy.loginfo(f"Moving to waypoint: {waypoint}")
            self.move_to_waypoint(waypoint[0], waypoint[1])
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        # Parse the start position from command-line arguments
        if len(sys.argv) < 3:
            print("Usage: rosrun your_package_name path_follower.py <start_x> <start_y>")
            sys.exit(1)
        
        start_x = float(sys.argv[1])
        start_y = float(sys.argv[2])
        
        # Example path starting from the given start position
        path = [(start_x, start_y), (0, 12), (0, 13), (0, 14), (0, 15), (0, 16), (1, 16), (2, 16), (3, 16), (4, 16), (5, 16), (6, 16), (7, 16), (8, 16), (9, 16), (10, 16), (11, 16), (12, 16), (13, 16), (14, 16), (15, 16), (16, 16), (17, 16), (18, 16), (19, 16), (19, 17), (19, 18), (19, 19), (20, 19), (21, 19), (22, 19), (23, 19), (24, 19), (25, 19), (26, 19), (27, 19), (28, 19), (29, 19), (30, 19), (31, 19), (32, 19), (33, 19), (34, 19), (35, 19), (36, 19), (37, 19), (38, 19), (39, 19), (40, 19), (41, 19), (42, 19), (43, 19), (44, 19), (45, 19), (46, 19), (47, 19), (48, 19), (49, 19), (50, 19), (51, 19), (52, 19), (53, 19), (54, 19), (55, 19), (56, 19), (57, 19), (58, 19), (59, 19), (60, 19), (61, 19), (62, 19), (63, 19), (64, 19), (65, 19), (66, 19), (67, 19), (68, 19), (69, 19), (70, 19), (71, 19), (72, 19), (73, 19), (73, 18), (73, 17), (74, 17), (75, 17), (76, 17), (77, 17), (78, 17), (79, 17), (80, 17), (81, 17), (82, 17), (83, 17), (84, 17), (85, 17), (86, 17), (87, 17), (88, 17), (89, 17), (90, 17), (91, 17), (91, 18), (91, 19), (91, 20), (91, 21), (91, 22), (91, 23), (91, 24), (91, 25), (91, 26), (91, 27), (91, 28), (91, 29), (91, 30), (91, 31), (91, 32), (91, 33), (91, 34), (91, 35), (91, 36), (91, 37), (90, 37), (89, 37), (88, 37), (87, 37), (86, 37), (85, 37), (84, 37), (83, 37), (82, 37), (81, 37), (80, 37), (79, 37), (78, 37), (77, 37), (76, 37), (75, 37), (74, 37), (73, 37), (73, 38), (73, 39), (73, 40), (73, 41), (73, 42), (73, 43), (73, 44), (73, 45), (73, 46), (73, 47), (73, 48), (73, 49), (73, 50), (73, 51), (73, 52), (73, 53), (73, 54), (73, 55), (72, 55), (71, 55), (70, 55), (69, 55), (68, 55), (67, 55), (66, 55), (65, 55), (64, 55), (63, 55), (62, 55), (61, 55), (60, 55), (59, 55), (58, 55), (57, 55), (56, 55), (55, 55), (55, 56), (55, 57), (55, 58), (55, 59), (55, 60), (55, 61), (55, 62), (55, 63), (55, 64), (55, 65), (55, 66), (55, 67), (55, 68), (55, 69), (55, 70), (55, 71), (55, 72), (55, 73), (54, 73), (53, 73), (52, 73), (51, 73), (50, 73), (49, 73), (48, 73), (47, 73), (46, 73), (45, 73), (44, 73), (43, 73), (42, 73), (41, 73), (40, 73), (39, 73), (38, 73), (37, 73), (37, 72), (37, 71), (37, 70), (36, 70), (35, 70), (34, 70), (33, 70), (32, 70), (31, 70), (30, 70), (29, 70), (28, 70), (27, 70), (26, 70), (25, 70), (24, 70), (23, 70), (22, 70), (21, 70), (20, 70), (19, 70), (18, 70), (17, 70), (16, 70), (16, 71), (16, 72), (16, 73), (16, 74), (16, 75), (16, 76), (16, 77), (16, 78), (16, 79), (16, 80), (16, 81), (16, 82), (16, 83), (16, 84), (16, 85), (16, 86), (16, 87), (16, 88), (16, 89), (16, 90), (16, 91), (16, 92), (16, 93), (16, 94), (16, 95), (16, 96), (16, 97), (16, 98), (16, 99), (16, 100), (16, 101), (16, 102), (16, 103), (16, 104), (16, 105), (16, 106), (17, 106), (18, 106), (18, 107), (19, 107), (20, 107), (21, 107), (22, 107), (23, 107), (24, 107), (25, 107), (26, 107), (27, 107)]  # Example path, customize it

        controller = PathFollower(start_x, start_y, path)
        controller.follow_path()
    except rospy.ROSInterruptException:
        pass
