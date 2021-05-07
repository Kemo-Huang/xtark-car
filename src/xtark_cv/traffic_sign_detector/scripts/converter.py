#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import rospy
import signal
from cv_bridge import CvBridge, CvBridgeError
import rosbag
import os

# def run():
#     with open(raw_data, 'rb') as f:
#         data = pickle.load(f)
#     cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     np.save(img_data, cv_image)


if __name__ == '__main__':
    # rospy.init_node('img_converter', anonymous=True)
    # bridge = CvBridge()
    # signal.signal(signal.SIGUSR1, run)
    # rospy.spin()
    raw_data = os.path.abspath('raw_data.pkl')
    img_data = os.path.abspath('color_img.npy')
    bridge = CvBridge()
    data =
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    np.save(img_data, cv_image)
