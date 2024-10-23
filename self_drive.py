#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import nav_msgs.msg

class PathPublisher:
    def __init__(self, file_path):
        self.publisher = rospy.Publisher("path", nav_msgs.msg.Path, queue_size=1)
        self.rate = rospy.Rate(1)  # Publish at 1 Hz
        self.waypoints = self.read_waypoints_from_file(file_path)

    def read_waypoints_from_file(self, file_path):
        waypoints = []
        try:
            with open(file_path, "r") as f:
                for line in f:
                    x, y, z = map(float, line.split(","))
                    pose = geometry_msgs.msg.PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = z
                    waypoints.append(pose)
            return waypoints
        except FileNotFoundError:
            rospy.logerr(f"File not found: {file_path}")
            return []
        except ValueError:
            rospy.logerr(f"Invalid file format: {file_path}")
            return []

    def publish_path(self):
        path = nav_msgs.msg.Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = self.waypoints
        for i, pose in enumerate(path.poses):
            pose.header.seq = i
        self.publisher.publish(path)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_path()
            self.rate.sleep()

if __name__ == '__main__':
    file_path = "/home/jetson5/catkin_ws/race.txt"
    rospy.init_node('path_publisher')
    path_publisher = PathPublisher(file_path)
    path_publisher.run()
