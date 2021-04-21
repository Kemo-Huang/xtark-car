#!/usr/bin/env python
from __future__ import print_function

import pytesseract
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class TrafficSignDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        # self.image_pub = rospy.Publisher("/traffic/image", Image, queue_size=1)
        self.light_color_pub = rospy.Publisher("/traffic/number", String, queue_size=1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cimg, numbers, = detect(cv_image)
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cimg, "bgr8"))
            self.light_color_pub.publish(numbers)
        except CvBridgeError as e:
            print(e)


def detect(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray = cv2.resize(gray, (300, 300))
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 1000, minRadius=0, maxRadius=1000)
    ret_str = ''
    if circles is not None:
        x, y, r = circles[0, 0]
        x = int(x)
        y = int(y)
        r = int(r)
        crop_img = gray[y - r: y + r, x - r: x + r]
        string = pytesseract.image_to_string(crop_img, config="--psm 6 digits")
        for s in string.split():
            if s.isdigit():
                ret_str += s
    return img, ret_str


def main():
    rospy.init_node('sign_detector', anonymous=True)
    TrafficSignDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
