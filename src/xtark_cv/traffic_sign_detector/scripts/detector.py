#!/usr/bin/env python3
from __future__ import print_function

import pytesseract
import sys
import rospy
import cv2
import numpy as np

class TrafficSignDetector:
    def __init__(self):
        pass


def main():
    image = cv2.imread('40.jpg')
    # print('normal', normal_speed_limit_detection(image))
    print('removed', removed_speed_limit_detection(image))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def digit_detection(image):
    string = pytesseract.image_to_string(cv2.resize(image, (100, 100)), config="--oem 3 --psm 9 outputbase digits")
    ret_str = ''.join([s for s in string.split() if s.isdigit()])
    return ret_str


def crop_image_by_circles(circles, vis_img, cropped_img):
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
        cv2.circle(vis_img, (x, y), r, (255, 0, 0), 2)
    cropped_img = cropped_img[int(min_ymr): int(max_ypr), int(min_xmr): int(max_xpr)]
    cv2.imshow('img_circles', vis_img)
    return vis_img, cropped_img


def normal_speed_limit_detection(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask = mask0 + mask1

    mask_img = image.copy()
    mask_img[mask == 0] = 0

    mask_gray = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(mask_gray, cv2.HOUGH_GRADIENT, 1, 20, param2=40, minRadius=10, maxRadius=800)
    cv2.imshow('mask_img', mask_img)
    cv2.imshow('mask_gray', mask_gray)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    if circles is not None:
        vis_img, thresh = crop_image_by_circles(circles, image.copy(), thresh)
        # publish
        if thresh is not None:
            cv2.imshow('final_img', cv2.resize(thresh, (100, 100)))
            return digit_detection(thresh)
    return ''


def removed_speed_limit_detection(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param2=100, minRadius=10, maxRadius=800)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
    if circles is not None:
        vis_img, thresh = crop_image_by_circles(circles, image.copy(), thresh)
    if thresh is not None:
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (25, 25))
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        inv_img = cv2.bitwise_not(opening)
        blur_img = cv2.medianBlur(inv_img, 25)

        cv2.imshow('inv_img', cv2.resize(inv_img, (100, 100)))
        cv2.imshow('blur_img', cv2.resize(blur_img, (100, 100)))

        return digit_detection(inv_img) + ' ' + digit_detection(blur_img)
    return ''


def limit_the_speed(image):
    pass


if __name__ == '__main__':
    main()
