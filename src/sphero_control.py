#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from maze_control.msg import IntList, Waypoints
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class sphero_finder:

    def __init__(self):

        self.waypnts = []
        self.outline = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imagecb)
        self.image_sub = rospy.Subscriber("/waypoints",Waypoints,self.waypntcb)
        self.image_pub = rospy.Publisher("center_point1",Point,queue_size=10)
        self.image_pub2 = rospy.Publisher("center_point2",Point,queue_size=10)
        self.prey_color_pub = rospy.Publisher("prey/set_color",ColorRGBA,queue_size=1)
        self.predator_color_pub = rospy.Publisher("predator/set_color",ColorRGBA,queue_size=1)

    def imagecb(self,data):
        try:

            # Convert Image message to CV image with blue-green-red color order (bgr8)
            # create trackbars for color change
            # cv2.namedWindow('Converted Image')
            # cv2.createTrackbar('Lower Hue','Converted Image',0,180,nothing)
            # cv2.createTrackbar('Lower Sat','Converted Image',0,255,nothing)
            # cv2.createTrackbar('Lower Value','Converted Image',0,255,nothing)
            # cv2.createTrackbar('Upper Hue','Converted Image',0,180,nothing)
            # cv2.createTrackbar('Upper Sat','Converted Image',0,255,nothing)
            # cv2.createTrackbar('Upper Value','Converted Image',0,255,nothing)
            # switch = '0 : OFF \n1 : ON'
            # cv2.createTrackbar(switch, 'Converted Image',0,1,nothing)

            # lowh = cv2.getTrackbarPos('Lower Hue','Converted Image')
            # lows = cv2.getTrackbarPos('Lower Sat','Converted Image')
            # lowv = cv2.getTrackbarPos('Lower Value','Converted Image')
            # upph = cv2.getTrackbarPos('Upper Hue','Converted Image')
            # upps = cv2.getTrackbarPos('Upper Sat','Converted Image')
            # uppv= cv2.getTrackbarPos('Upper Value','Converted Image')

            # lower_red = np.array([lowh,lows,lowv])
            # upper_red = np.array([upph,upps,uppv])
            lower_red = np.array([140,75,150])
            upper_red = np.array([180,170,255])
            lower_red2 = np.array([70,90,175])
            upper_red2 = np.array([130,190,255])
            # lower_red2 = np.array([0,180,125])
            # upper_red2 = np.array([10,255,255])

            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # img_original = cv2.flip(img_original,1)
            hsv = cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,lower_red, upper_red)
            mask2 = cv2.inRange(hsv,lower_red2, upper_red2)
            #res =cv2.bitwise_and(img_original,img_original,mask= mask)

            contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            contour2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            if len(contour) > 0:
                c = max(contour, key = cv2.contourArea)
                ((x,y),radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if radius > 5:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # else:
                #   center = (0,0)
                    # res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),(0,255,0),2)
                    img_original = cv2.circle(img_original,(int(center[0]),int(center[1])),int(radius),(0,255,0),2)

                  #print center
                    self.image_pub.publish(center[0],center[1],0)

            if len(contour2) > 0:
                c = max(contour2, key = cv2.contourArea)
                ((x,y),radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if radius > 5:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # else:
                #   center = (0,0)
                    #res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),(0,0,255),2)
                    img_original = cv2.circle(img_original,(int(center[0]),int(center[1])),int(radius),(255,0,0),2)

                  #print center
                    self.image_pub2.publish(center[0],center[1],0)

            if len(self.waypnts) > 0:
                for wp in self.waypnts:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),-1)

                pts = np.array(self.outline,np.int32)
                pts = pts.reshape((-1,1,2))
                cv2.polylines(img_original,[pts],True,(0,255,0),3)
                # cv2.drawContours(img_original, np.array(self.outline), -1, (0, 255, 0), 3)

                for wp in self.outline:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(255,0,0),-1)

            cv2.imshow("Converted Image",img_original)#np.hstack([img_original,res]))
            #cv2.imshow("Converted Image",np.hstack([img_original,res]))

            cv2.waitKey(3)
        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

    def waypntcb(self,data):
        alist = []
        outln = []
        for i in range(len(data.data)):
            if i < len(data.data)-6:
                alist.append(data.data[i].data)
            else:
                outln.append(data.data[i].data)
        self.waypnts = alist
        self.outline = outln
        # print alist


def nothing(x):
    pass

def main():
    rospy.init_node('sphero_finder', anonymous=True)
    ic = sphero_finder()
    rospy.sleep(1)
    ic.prey_color_pub.publish(ColorRGBA(1,0,0,1))
    ic.predator_color_pub.publish(ColorRGBA(0,0,1,1))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    # rospy.init_node('main')
    # rospy.Subscriber("/waypoints",Waypoints,data_cb)

    # rospy.spin()

if __name__ == "__main__":
    main()
