#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebcamNode:
    def __init__(self):
        self.cap = None
        self.current_index = 1  # Start with camera index 1
        self.open_camera(self.current_index)

        if self.cap is None or not self.cap.isOpened():
            rospy.logerr("Error: Camera not opened")
            rospy.signal_shutdown("Camera not opened")
            return

        rospy.loginfo("Camera opened successfully")
        self.pub = rospy.Publisher('/webcam_image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.running = True

    def open_camera(self, index):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logwarn(f"Camera index {index} could not be opened. Trying next index...")
            if index == 1:
                self.current_index = 0  # Switch to index 0
                self.open_camera(self.current_index)

    def capture_and_publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Failed to read frame. Attempting to switch camera index...")
            self.switch_camera()
            return

        # Resize the frame for faster processing (optional)
        frame = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_AREA)
        # Convert the frame to a ROS Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)

    def switch_camera(self):
        # Attempt to switch to the other camera index
        self.cap.release()  # Release the current camera
        self.current_index = 1 if self.current_index == 0 else 0  # Toggle index
        self.open_camera(self.current_index)  # Try to open the new index

    def start(self):
        rospy.init_node('webcam_node')
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and self.running:
            self.capture_and_publish_frame()
            rate.sleep()  # Maintain the loop rate

    def cleanup(self):
        self.running = False
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            rospy.loginfo("Camera released")

if __name__ == '__main__':
    node = WebcamNode()
    try:
        node.start()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()
