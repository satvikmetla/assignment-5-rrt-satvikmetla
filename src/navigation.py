#!/usr/bin/env python3

import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rrt import RRT
import time

class Navigator:
    def __init__(self):
        rospy.init_node('bot_navigation', anonymous=True)

        grid_path = rospy.get_param('/grid','/home/cse4568/catkin_ws/src/assignment-5-rrt-satvikmetla/map/map.npy')
        self.goal = rospy.get_param('/goal_position',[2,-1.5])
        self.grid = np.load(grid_path)
        self.origin = (-3,3)
        self.resolution = 0.05
        self.tolerance = 0.1

        self.count = 1
        self.path = []
        
        # Initialize robot state
        self.position = Point()
        self.orientation = 0

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.loop_rate = rospy.Rate(10)

    def grid_to_world(self,x, y):
        wx = self.origin[0] + x * self.resolution
        wy = self.origin[1] - y * self.resolution
        return (wx, wy)

    def world_to_grid(self,wx, wy):
        x = int((wx - self.origin[0]) / self.resolution)
        y = int((self.origin[1] - wy) / self.resolution)
        return (x, y)

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.orientation = euler[2]  # Extract yaw from quaternion

        self.count += 1

    def get_path(self):
        sx,sy = self.world_to_grid(self.position.x,self.position.y)
        gx,gy = self.world_to_grid(self.goal[0],self.goal[1])
        rrt = RRT((sx,sy),(gx,gy),self.grid,step_size=10)
        path_ = rrt.path_finder()
        for point_ in path_:
            self.path.append(self.grid_to_world(point_[0],point_[1]))
        print(self.path)

    def compute_distance_and_angle(self, goal_x, goal_y):
        distance = math.sqrt((goal_x - self.position.x)**2 + (goal_y - self.position.y)**2)
        angle_to_goal = math.atan2(goal_y - self.position.y, goal_x - self.position.x)
        return distance, angle_to_goal
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.count == 1:
            rospy.loginfo("Waiting for odometry...")
            rate.sleep()

        self.get_path()
        twist = Twist()
        for im_goal in self.path:
            reached = False
            while not rospy.is_shutdown() and not reached:
                dis, angle = self.compute_distance_and_angle(im_goal[0], im_goal[1])
                if dis < self.tolerance:
                    reached = True
                    break
                angular_error = self.normalize_angle(angle - self.orientation)
                twist.linear.x = min(0.15, dis)
                twist.angular.z = 0.85 * angular_error

                self.velocity_publisher.publish(twist)
                rate.sleep()
            print("Intermediate GOAL REACHED",im_goal)
        # Stop
        print("Final goal reached . Stopping bot...")
        twist.linear.x = 0
        twist.angular.z = 0
        self.velocity_publisher.publish(twist)



if __name__ == '__main__':
    try:
        navigator = Navigator()
        navigator.main_loop()
    except rospy.ROSInterruptException:
        pass


