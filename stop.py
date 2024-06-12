#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class LidarSubscriber:

    def __init__(self):
        rospy.init_node('lidar_subscriber_publisher', anonymous=True)

        self.subscription = rospy.Subscriber(
            'scan',
            LaserScan,
            self.listener_callback,
            queue_size=100)

        self.publisher_ = rospy.Publisher('lidar_pimp', Bool, queue_size=10)
        self.i = 0
        self.tolerance = 0.0872 * 33  # 5 degrees * 24, 120 degrees
        self.target_angle = 0
        self.obstacle_array = [False, False, False, False, False]  # Initialize with no obstacles

    def obstacles(self, arr):
        all_sectors = []
        for sector in arr:
            count = 0
            for point in sector:
                if point < 0.15:
                    count += 1
            all_sectors.append(count >= 3)
        return all_sectors

    def listener_callback(self, msg):
        data = []
        start_index = int(round((self.target_angle - self.tolerance) / msg.angle_increment))
        end_index = int(round((self.target_angle + self.tolerance) / msg.angle_increment))
        for i in range(start_index, end_index):
            data.append(round(msg.ranges[i], 2))

        # Move array splitting outside the loop
        data = np.array_split(data, [5, 7, 10, 12])

        self.obstacle_array = self.obstacles(data)

        # Debugging: Print obstacle array values
        for i, obstacle in enumerate(self.obstacle_array):
            print(f"Sector {i}: {obstacle}")

        # Create Bool message
        obstacle_msg = Bool(data=True in self.obstacle_array)
        self.publisher_.publish(obstacle_msg)
      
if __name__ == '__main__':
    lidar_subscriber = LidarSubscriber()
    rospy.spin()
