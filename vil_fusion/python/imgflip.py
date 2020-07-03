#!/usr/bin/env python2

"""
SuperMegaBot bags had the image inverted. Rovio expects it not inverted. This was the quickest way to get them
right-side-up. There was probably a better way involving just a static transform.
"""

from __future__ import division, print_function

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

node = rospy.init_node("imgflip")

bridge = CvBridge()
pub = rospy.Publisher("/cam0/image_flipped", Image)


def img_callback(img):
    cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    h, w = cv_img.shape[0:2]
    m = cv2.getRotationMatrix2D((w / 2, h / 2), 180, 1)
    rotated = cv2.warpAffine(cv_img, m, (w, h))
    new_img = bridge.cv2_to_imgmsg(rotated)
    new_img.header = img.header
    pub.publish(new_img)


sub = rospy.Subscriber("/cam0/image_rect", Image, callback=img_callback)

if __name__ == "__main__":
    rospy.spin()
