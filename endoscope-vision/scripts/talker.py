#!/usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)

rospy.init_node("endo_vision", anonymous=True)
image_pub = rospy.Publisher("webcam_image", Image, queue_size=1)

bridge = CvBridge()

while not rospy.is_shutdown():
    ret, cv_image = cap.read()
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))

cap.release()
cv2.destroyAllWindows()

