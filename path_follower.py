!/usr/bin/env python3

import nav_msgs.msg
import geometry_msgs.msg
import rospy
import math
import numpy as np
from std_msgs.msg import Bool

class PathFollower:
    def __init__(self):
        self.path_subscriber = rospy.Subscriber("path", nav_msgs.msg.Path, self.path_callback)
        self.gps_subscriber = rospy.Subscriber("gps_enu", geometry_msgs.msg.PoseStamped, self.gps_callback)
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.lidar_subscriber = rospy.Subscriber("lidar_pimp", Bool, self.lidar_callback)
        self.lidar_status = False
        self.rate = rospy.Rate(10)  # Publish at 10 Hz
        self.current_waypoint = 0
        self.target_pose = None

        self.vehicle_state = np.array([
            [0.0],
            [0.0],
            [0.0]
        ])

        self.bumper = 44.0
        self.width = 88.0
        self.wheelbase = 70.0
        self.update_rate = 0.01

        self.gps_x_prev = None
        self.gps_y_prev = None
        self.path = None

    def lidar_callback(self, msg):
        self.lidar_status = msg.data

    def path_callback(self, path_msg):
        if self.path is None:
            self.path = path_msg
        else:
            self.path.poses.extend(path_msg.poses)

        if self.current_waypoint >= len(self.path.poses):
            self.current_waypoint = 0

        self.target_pose = self.path.poses[self.current_waypoint]

        # Update the current pose based on the robot's current position
        self.current_pose = self.vehicle_state[0:3]

        print("Current waypoint:", self.current_waypoint)
        print("Target pose:", self.target_pose.pose.position.x, self.target_pose.pose.position.y)
        print("Current pose:", self.current_pose[0], self.current_pose[1])

        # Calculate the difference between current and target poses
        delta_x = self.target_pose.pose.position.x - self.current_pose[0]
        delta_y = self.target_pose.pose.position.y - self.current_pose[1]

        print("Delta x:", delta_x, "Delta y:", delta_y)

        # Calculate angular velocity based on the difference in heading
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y
        current_x = self.current_pose[0]
        current_y = self.current_pose[1]

        target_heading = math.atan2(target_y - current_y, target_x - current_x)
        current_heading = math.atan2(self.current_pose[1] - current_y, self.current_pose[0] - current_x)
        diff = (target_heading - current_heading) % (2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi

        linear_vel = float(1.5)
        if delta_x > 0:
            linear_vel = abs(linear_vel)
        else:
            linear_vel = -abs(linear_vel)

        angular_vel = diff

        cmd_vel_msg = geometry_msgs.msg.Twist()

        if self.lidar_status:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
        else:
            cmd_vel_msg.linear.x = float(linear_vel)
            cmd_vel_msg.angular.z = float(angular_vel)

        self.cmd_vel_publisher.publish(cmd_vel_msg)

        print("Target heading:", target_heading)
        print("Current heading:", current_heading)
        print("Heading difference:", diff)
        print("Linear velocity: ", linear_vel)
        print("Angular velocity: ", angular_vel)

    def gps_callback(self, gps_msg):
        if self.gps_x_prev is None or self.gps_y_prev is None:
            if self.gps_x_prev is None or self.gps_y_prev is None:
            self.gps_x_prev = gps_msg.pose.position.x
            self.gps_y_prev = gps_msg.pose.position.y
        print("GPS Coordinates:")
        print("X:", gps_msg.pose.position.x)
        print("Y:", gps_msg.pose.position.y)
        print("Orientation:", gps_msg.pose.orientation.z)
        self.vehicle_state[0] = gps_msg.pose.position.x
        self.vehicle_state[1] = gps_msg.pose.position.y
        self.vehicle_state[2] = gps_msg.pose.orientation.z
        self.current_pose = self.vehicle_state[0:3]

    def control_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_follower")
    follower = PathFollower()
    follower.control_loop()
