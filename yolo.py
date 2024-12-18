#!/usr/bin/env python3

import cv2
import rospy
import torch
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class YOLOv8Detector:
    def __init__(self):
        rospy.loginfo("Initializing YOLOv8Detector...")
        self.model = YOLO('yolov8n.pt')  # Load the YOLOv8 model
        rospy.loginfo("Model loaded successfully!")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/webcam_image', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/yolov8_image', Image, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #self.lidar_subscriber = rospy.Subscriber("lidar", Bool, self.lidar_callback)
        #self.lidar_status = False

        # Load camera parameters directly from calibration
        self.mtx = np.array([[997.9850778899299, 0.0, 571.2404286262511],
                             [0.0, 1001.0991379680671, 310.1803177420431],
                             [0.0, 0.0, 1.0]])

        self.dist = np.array([-0.5822967795072609, 0.3367466590671402,
                              -0.011294357806644038, -0.05117747006198482, 0.0])

        self.KNOWN_HEIGHT = 0.46  # Adjust based on the object
        self.KNOWN_WIDTH = 0.20    # Adjust based on the object

    #def lidar_callback(self, msg):
        #self.lidar_status = msg.data

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return

        # Undistort the image using the camera matrix and distortion coefficients
        img = cv2.undistort(img, self.mtx, self.dist)
        img_tensor = cv2.resize(img, (640, 640)).transpose((2, 0, 1)).astype(np.float32) / 255.0
        img_tensor = torch.from_numpy(img_tensor).unsqueeze(0)

        results = self.model(img_tensor)

        bottom_threshold = img.shape[0] - 100

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, confidence, class_id = result
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            bounding_box_height = y2 - y1
            bounding_box_width = x2 - x1

            stop_robot = False
            # Check if bounding box height is reasonable
            if bounding_box_height > 0:
                # Calculate distance based on bounding box height and width
                focal_length = self.mtx[1, 1]
                distance = (self.KNOWN_HEIGHT * focal_length) / bounding_box_height

                if bounding_box_width > 0:
                    distance_width = (self.KNOWN_WIDTH * focal_length) / bounding_box_width
                    distance = (distance + distance_width)

                rospy.loginfo(f"Bounding Box Height: {bounding_box_height}, Width: {bounding_box_width}, Distance: {distance:.2f}m")

                # Set a minimum distance threshold
                if (0 <= x1 <= 640) and (340 <= y1 <= 640):
                    distance = 1.5

                cv2.putText(img, f'Distance: {distance:.2f}m', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # Robot control logic based on distance
                if distance < 2.0 or (x1 < img.shape[1] // 2 < x2):
                    stop_robot = True

        # Publish the robot 's movement command
        stop_cmd = Twist()
        if stop_robot:
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
        else:
            stop_cmd.linear.x = 0.5
            stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

        # Publish the image to the ROS topic
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_pub.publish(image_msg)

if __name__ == '__main__':
    rospy.init_node('yolov8_detector')
    detector = YOLOv8Detector()
    rospy.spin()
