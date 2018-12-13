#!/usr/bin/env python
# USAGE
# python realtime_stitching.py

# import the necessary packages
from __future__ import print_function
from pyimagesearch.panorama import Stitcher
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import datetime
import imutils
import time
import cv2
import rospy
import cv2

class combine_images:

    def __init__(self):
        # initialize the image stitcher, motion detector, and total
        # number of frames read
        self.stitcher = Stitcher()
        self.check_left = False
        self.bridge=CvBridge()
        self.keytime = 500
        self.left = False
        self.right = False
        self.image_left_sub = rospy.Subscriber("/usb_cam1/image_raw",Image,self.image_leftcb)
        self.image_right_sub = rospy.Subscriber("/usb_cam2/image_raw",Image,self.image_rightcb)
        self.image_combine_pub = rospy.Publisher("combined_image",Image,queue_size=1)
        self.image_warped_pub = rospy.Publisher("warped_image",Image,queue_size=1)

    def image_leftcb(self,data):
        try:
            left = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.left = imutils.rotate_bound(left, -90)

            # self.left = imutils.resize(left, width=400)
            self.check_left = True
        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

    def image_rightcb(self,data):
        try:
            right = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.right = imutils.rotate_bound(right, -90)
            # self.right = imutils.resize(right, width=400)
            if self.check_left == True:
                self.process_image([self.left,self.right])
        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

    def process_image(self,data):
        left,right = data
        # stitch the frames together to form the panorama
        # IMPORTANT: you might have to change this line of code
        # depending on how your cameras are oriented; frames
        # should be supplied in left-to-right order
        if self.stitcher.isCal == True:
            showMatch = False
        else:
            showMatch = True

        res = self.stitcher.stitch([left, right],showMatches=showMatch)# no homograpy could be computed

        if len(res) == 1:
            result = res[0]
        elif len(res) == 2 and showMatch == True:
            showviz = True
            result,vis = res
        else:
            result, imageA = res
            showviz = False
        if result is None:
            print("[INFO] homography could not be computed")
            return

        # show the output images
        cv2.imshow("Result", result)
        if self.stitcher.isCal == False:
            if showviz == True:
                cv2.imshow("Keypoint Matches",vis)
            cv2.imshow("Left Frame", self.left)
            cv2.imshow("Right Frame", self.right)


        else:

            msg_frame = self.bridge.cv2_to_imgmsg(result,"bgr8")
            msg_warped = self.bridge.cv2_to_imgmsg(imageA,"bgr8")
            # cv2.imshow("ImageA", imageA)
            # cv2.imshow("ImageB", imageB)
            try:
                self.image_combine_pub.publish(msg_frame)
            except CvBridgeError as e:
                print(e)
            try:
                self.image_warped_pub.publish(msg_warped)
            except CvBridgeError as e:
                print(e)

        key = cv2.waitKey(self.keytime) #& 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            rospy.signal_shutdown("[INFO] cleaning up...")
        if key == ord("c"):
            self.stitcher.isCal = True
            self.keytime = 1
            if showviz == True:
                cv2.destroyWindow("Keypoint Matches")
            cv2.destroyWindow("Left Frame")
            cv2.destroyWindow("Right Frame")

def main():
    rospy.init_node('combine_images', anonymous=False)
    print("[INFO] starting cameras...")
    ic = combine_images()
    rospy.sleep(2)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
