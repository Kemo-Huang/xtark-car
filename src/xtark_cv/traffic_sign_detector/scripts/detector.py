#!/usr/bin/env python3
from __future__ import print_function

import pytesseract
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import subprocess
import os
import time
import rosbag


class TrafficSignDetector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.img_callback)
        self.normal_img_pub = rospy.Publisher("/traffic_sign/normal_image", Image, queue_size=1)
        self.blur_img_pub = rospy.Publisher("/traffic_sign/blur_image", Image, queue_size=1)
        self.string_pub = rospy.Publisher("/traffic_sign/string", String, queue_size=1)
        self.bool_pub = rospy.Publisher("/traffic_sign/speed_limit", Bool, queue_size=1)
        self.speed_limit = False
        self.timer = 0

    def img_callback(self, data):
        bag = rosbag.Bag('raw.bag', 'w')
        bag.write('raw', data)
        bag.close()
        subprocess.Popen(["/usr/bin/python", os.path.abspath("converter.py")])
        print('callback')
        while True:
            if os.path.exists('color_img.npy'):
                print('file exists')
                cv_image = np.load('color_img.npy')
                break
        os.remove('color_img.npy')
        print(cv_image.shape)
        if self.normal_speed_limit_detection(cv_image):
            self.speed_limit = True
            self.timer = time.time()
        elif self.removed_speed_limit_detection(cv_image) and time.time() - self.timer > 5:
            self.speed_limit = False
        self.bool_pub.publish(self.speed_limit)

    def normal_speed_limit_detection(self, bgr_img):
        is_detected = False

        # get red mask
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)
        lower_red = np.array([160, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask = mask0 + mask1

        mask_img = bgr_img.copy()
        mask_img[mask == 0] = 0

        mask_gray = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(mask_gray, cv2.HOUGH_GRADIENT, 1, 100, param2=40, minRadius=30, maxRadius=300)
        if circles is not None:
            gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
            thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            thresh = crop_image_by_circles(circles, thresh)
            # publish
            # self.normal_img_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "mono8"))
            if '10' in digit_detection(thresh) and thresh.shape[0] > 35 and thresh.shape[1] > 35:
                self.string_pub.publish('red: ' + str(thresh.shape))
                is_detected = True
        return is_detected

    def removed_speed_limit_detection(self, bgr_img):
        is_detected = False
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param2=150, minRadius=30, maxRadius=300)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        if circles is not None:
            thresh = crop_image_by_circles(circles, thresh)

            # remove lines
            kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (13, 13))
            opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
            inv_img = cv2.bitwise_not(opening)
            blur_img = cv2.medianBlur(inv_img, 5)

            # publish
            # self.blur_img_pub.publish(self.bridge.cv2_to_imgmsg(blur_img, "mono8"))
            if '10' in digit_detection(blur_img):
                self.string_pub.publish('remove: ' + str(blur_img.shape))
                is_detected = True
        return is_detected


def digit_detection(image):
    string = pytesseract.image_to_string(cv2.resize(image, (100, 100)), config="--oem 3 --psm 9 outputbase digits")
    ret_str = ''.join([s for s in string.split() if s.isdigit()])
    return ret_str


def crop_image_by_circles(circles, cropped_img):
    circles = circles.astype(np.int)
    circles = circles[0]
    max_ypr = max_xpr = 0
    min_ymr = min_xmr = np.inf
    scale = 0.7
    img_size = cropped_img.shape
    for i in range(len(circles)):
        # find the smallest rectangle that covers all the circles
        x, y, r = circles[i]
        if 0 < y - scale * r < min_ymr:
            min_ymr = y - scale * r
        if 0 < x - scale * r < min_xmr:
            min_xmr = x - scale * r
        if max_ypr < y + scale * r < img_size[0]:
            max_ypr = y + scale * r
        if max_xpr < x + scale * r < img_size[1]:
            max_xpr = x + scale * r
    cropped_img = cropped_img[int(min_ymr): int(max_ypr), int(min_xmr): int(max_xpr)]

    return cropped_img


if __name__ == '__main__':
    rospy.init_node('traffic_sign_detector', anonymous=True)
    TrafficSignDetector()
    rospy.spin()
