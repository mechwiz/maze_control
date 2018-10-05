#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from scipy import stats
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from maze_control.msg import IntList, Waypoints


def data_cb(data):
    alist = []
    for i in range(len(data.data)):
        alist.append(data.data[i].data)
    print alist

def main():
    rospy.init_node('main')
    rospy.Subscriber("/waypoints",Waypoints,data_cb)

    rospy.spin()

if __name__ == "__main__":
    main()
