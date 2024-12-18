#!/usr/bin/env python3

import nav_msgs.msg
import geometry_msgs.msg
import rospy
import math
import numpy as np
import sensor_msgs.msg
from tf.transformations import euler_from_quaternion

class PathFollower:
    def __init__(self):
        rospy.init_node("path_follower")

        # Subscribers and Publisher
        self.path_subscriber = rospy.Subscriber("path", nav_msgs.msg.Path, self.path_callback)
        self.gps_subscriber = rospy.Subscriber("gps_enu", geometry_msgs.msg.PoseStamped, self.gps_callback)
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
        self.imu_subscriber = rospy.Subscriber("/mavros/imu/data", sensor_msgs.msg.Imu, self.imu_callback)

        self.rate = rospy.Rate(10)
        self.target_pose = None
        self.path = None
        self.waypoint_index = 0
        self.current_yaw = 0.0
        self.gps_position = None

        # PID parameters
        self.kp = 1.0  # Proportional gain for steering
        self.base_speed = 0.5  # Base speed for moving forward
        self.heading_threshold = 10.0  # Threshold for heading adjustment in degrees

    def path_callback(self, path_msg):
        if not self.path:  # Simplified check for path initialization
            self.path = path_msg
            closest_index = self.find_closest_waypoint()
            self.waypoint_index = closest_index
            self.target_pose = self.path.poses[closest_index]
            rospy.loginfo(f"Path Received: {len(self.path.poses)} waypoints, starting at index {self.waypoint_index}")

    def imu_callback(self, data):
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = math.degrees(self.current_yaw)

    def gps_callback(self, data):
        # Store GPS position as a tuple for easier access later
        self.gps_position = (data.pose.position.x, data.pose.position.y)

    def calculation(self):
        if not all([self.path, self.target_pose, self.gps_position]):
            rospy.logwarn("Waiting for path, target pose, or GPS position.")
            return
            
        current_pose = self.gps_position
        closest_index = self.find_closest_waypoint()
        closest_pose = self.path.poses[closest_index].pose.position

        next_index = (closest_index + 1) % len(self.path.poses)
        next_pose = self.path.poses[next_index].pose.position

        # Calculate distance to the closest waypoint
        distance_to_target = math.sqrt((closest_pose.x - current_pose[0])**2 + (closest_pose.y - current_pose[1])**2)
        rospy.loginfo(f"Current Waypoint Index: {self.waypoint_index}, Distance to Target Waypoint: {distance_to_target}")

        # Control signal calculation using Stanley control method
        control_signal = self.stanley_control(current_pose[0], current_pose[1], closest_pose.x, closest_pose.y, next_pose.x, next_pose.y)

        # Calculate heading to the target waypoint and heading error
        target_angle = math.degrees(math.atan2(next_pose.y - closest_pose.y, next_pose.x - closest_pose.x))
        heading_error = self.calculate_heading_error(target_angle, self.current_yaw)

        # Adjust motor speeds based on heading error and publish command velocity
        angular_velocity = control_signal[0] if abs(heading_error) <= self.heading_threshold else max(min(self.kp * heading_error, 0.2), -0.2)

        cmd_vel_msg = geometry_msgs.msg.Twist()
        cmd_vel_msg.linear.x = self.base_speed
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Update waypoint index if the vehicle has reached the current target
        if distance_to_target < 1.0:
            self.waypoint_index = (self.waypoint_index + 1) % len(self.path.poses)
            self.target_pose = self.path.poses[self.waypoint_index]
            rospy.loginfo(f"Next waypoint: {self.target_pose.pose.position.x}, {self.target_pose.pose.position.y}")

    def find_closest_waypoint(self):
        if not self.gps_position:
            rospy.logwarn("GPS position not available.")
            return 0

        min_distance = float('inf')
        closest_waypoint_index = 0
        current_pose = self.gps_position

        for i, pose in enumerate(self.path.poses):
            distance = math.sqrt((pose.pose.position.x - current_pose[0])**2 + (pose.pose.position.y - current_pose[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_waypoint_index = i

        return closest_waypoint_index

    def calculate_heading_error(self, target_angle, current_yaw):
        error = (target_angle - current_yaw + 180) % 360 - 180  # Normalize to [-180, 180]
        return error

    def reached_waypoint(self, target_pose):
        current_pose = self.gps_position
        distance_to_target = math.sqrt((target_pose.pose.position.x - current_pose[0])**2 +
                                        (target_pose.pose.position.y - current_pose[1])**2)

        return distance_to_target < 1.0  # Threshold for reaching the waypoint

    def stanley_control(self, current_x, current_y, target_x, target_y, next_x, next_y):
        dx = target_x - current_x
        dy = target_y - current_y

        abs_distance = np.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)

        yaw_rad = math.radians(self.current_yaw)

        crosstrack_error = np.sin(target_angle - yaw_rad) * abs_distance

        dx_next = next_x - current_x
        dy_next = next_y - current_y

        target_angle_next = math.atan2(dy_next, dx_next)

        k_s = 0.5  # Steering gain constant

        heading_error = target_angle_next - yaw_rad

        crosstrack_steering_error = np.arctan2(0.5 * crosstrack_error, k_s * self.base_speed)

        steering_angle = crosstrack_steering_error + heading_error

        return steering_angle,

    def run(self):
        while not rospy.is_shutdown():
            self.calculation()
            self.rate.sleep()

if __name__ == "__main__":
    path_follower = PathFollower()
    path_follower.run()
