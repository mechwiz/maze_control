#!/usr/bin/env python
import rospy
import os, rospkg
import cv2
import numpy as np
import csv

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from maze_control.msg import IntList, Waypoints
from maze_control.srv import waypoint
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class sphero_finder:

    def __init__(self):
        self.raw_waypnts = Waypoints()
        self.waypnts = []
        self.waypnt_dict = {}
        self.prey_path = []
        self.prey_pathpnt = []
        self.predator_path = []
        self.predator_pathpnt = []
        self.outline = []
        self.warpedpic = None
        self.warped_prey = []
        self.warped_predator = []
        self.bridge = CvBridge()
        self.lower_green = np.array(rospy.get_param('spheromini_finder/lower_prey'))
        self.upper_green = np.array(rospy.get_param('spheromini_finder/upper_prey'))
        self.lower_red = np.array(rospy.get_param('spheromini_finder/lower_predator'))
        self.upper_red = np.array(rospy.get_param('spheromini_finder/upper_predator'))
        self.image_sub = rospy.Subscriber("/combined_image",Image,self.imagecb)
        self.image_sub2 = rospy.Subscriber("/warped_image",Image,self.warpedcb)
        self.waypnts_sub = rospy.Subscriber("/waypoints",Waypoints,self.waypntcb)
        self.image_pub = rospy.Publisher("center_point1",Point,queue_size=1)
        self.image_pub2 = rospy.Publisher("center_point2",Point,queue_size=1)
        self.prey_color_pub = rospy.Publisher("prey/set_color",ColorRGBA,queue_size=1)
        self.predator_color_pub = rospy.Publisher("predator/set_color",ColorRGBA,queue_size=1)

    def waypnt_srv(self,req):
        if len(self.waypnts)>0:
            rospy.loginfo('Points Captured')
            return self.raw_waypnts

    def warpedcb(self,data):
        try:
            # lower_green = np.array([1,125,255])
            # upper_green = np.array([100,255,255])
            # lower_red = np.array([133,30,250])
            # upper_red = np.array([153,210,255])

            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,self.lower_green, self.upper_green)
            mask2 = cv2.inRange(hsv,self.lower_red, self.upper_red)
            # res =cv2.bitwise_and(img_original,img_original,mask= mask2)

            contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            contour2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            if len(contour) > 0 and len(self.waypnts)>0:
                for c in contour:
                    ((x,y),radius) = cv2.minEnclosingCircle(c)
                    if cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False)>0:
                        # M = cv2.moments(c)
                        if radius > 3:
                            # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                            # res = cv2.circle(res,(int(x),int(y)),int(radius),(0,255,0),2)
                            # img_original = cv2.circle(img_original,(int(x),int(y)),int(radius),(0,255,0),2)

                            self.warped_prey = [int(x),int(y)]
                            break

            else:
                self.warped_prey = []

            if len(contour2) > 0 and len(self.waypnts)>0:
                for c in contour2:
                    ((x,y),radius) = cv2.minEnclosingCircle(c)
                    if cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False)>0:
                        # M = cv2.moments(c)
                        if radius > 3:
                            # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                            # res = cv2.circle(res,(int(x),int(y)),int(radius),(255,0,0),2)
                            # img_original = cv2.circle(img_original,(int(x),int(y)),int(radius),(255,0,0),2)

                            self.warped_predator = [int(x),int(y)]
                            break
            else:
                self.warped_predator = []

            # self.warpedpic = img_original

        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

    def imagecb(self,data):
        try:

            # Convert Image message to CV image with blue-green-red color order (bgr8)
            # create trackbars for color change
            cv2.namedWindow('Converted Image')
            cv2.createTrackbar('Lower Hue','Converted Image',self.lower_green[0],180,nothing)
            cv2.createTrackbar('Lower Sat','Converted Image',self.lower_green[1],255,nothing)
            cv2.createTrackbar('Lower Value','Converted Image',self.lower_green[2],255,nothing)
            cv2.createTrackbar('Upper Hue','Converted Image',self.upper_green[0],180,nothing)
            cv2.createTrackbar('Upper Sat','Converted Image',self.upper_green[1],255,nothing)
            cv2.createTrackbar('Upper Value','Converted Image',self.upper_green[2],255,nothing)
            switch = '0 : OFF \n1 : ON'
            cv2.createTrackbar(switch, 'Converted Image',0,1,nothing)

            lowh = cv2.getTrackbarPos('Lower Hue','Converted Image')
            lows = cv2.getTrackbarPos('Lower Sat','Converted Image')
            lowv = cv2.getTrackbarPos('Lower Value','Converted Image')
            upph = cv2.getTrackbarPos('Upper Hue','Converted Image')
            upps = cv2.getTrackbarPos('Upper Sat','Converted Image')
            uppv= cv2.getTrackbarPos('Upper Value','Converted Image')

            lower_green = np.array([lowh,lows,lowv])
            upper_green = np.array([upph,upps,uppv])

            # lower_green = np.array([1,125,255])
            # upper_green = np.array([100,255,255])
            # lower_red = np.array([133,30,250])
            # upper_red = np.array([153,210,255])

            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,lower_green, upper_green)
            mask2 = cv2.inRange(hsv,self.lower_red, self.upper_red)
            res =cv2.bitwise_and(img_original,img_original,mask= mask)

            contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            contour2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            if len(self.waypnts)>0:
                if len(contour) > 0:
                    for c in contour:
                        ((x,y),radius) = cv2.minEnclosingCircle(c)
                        if cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False)>0:
                            # M = cv2.moments(c)

                            if radius > 3:
                                # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                                res = cv2.circle(res,(int(x),int(y)),int(radius),(0,255,0),2)
                                img_original = cv2.circle(img_original,(int(x),int(y)),int(radius),(0,255,0),2)

                              #print center
                                self.image_pub.publish(int(x),int(y),0)
                                break
                elif len(self.warped_prey)>0:
                    img_original = cv2.circle(img_original,(self.warped_prey[0],self.warped_prey[1]),5,(0,255,0),2)
                    self.image_pub.publish(self.warped_prey[0],self.warped_prey[1],0)

            if len(self.waypnts)>0:
                if len(contour2) > 0:
                    for c in contour2:
                        ((x,y),radius) = cv2.minEnclosingCircle(c)
                        if cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False)>0:
                            # M = cv2.moments(c)

                            if radius > 3:
                                # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                                # res = cv2.circle(res,(int(x),int(y)),int(radius),(255,0,0),2)
                                img_original = cv2.circle(img_original,(int(x),int(y)),int(radius),(255,0,0),2)
                                self.image_pub2.publish(int(x),int(y),0)
                                break

                elif len(self.warped_predator)>0:
                        img_original = cv2.circle(img_original,(self.warped_predator[0],self.warped_predator[1]),5,(255,0,0),2)
                        self.image_pub2.publish(self.warped_predator[0],self.warped_predator[1],0)

            if len(self.waypnts) > 0:
                for wp in self.waypnts:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),-1)

                pts = np.array(self.outline,np.int32)
                cv2.polylines(img_original,[pts],True,(0,255,0),3)

                for wp in self.outline:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(255,0,0),-1)

            if len(self.prey_pathpnt) > 0:
                cv2.polylines(img_original,[self.prey_pathpnt],False,(255,255,0),2)

            if len(self.predator_pathpnt) > 0:
                cv2.polylines(img_original,[self.predator_pathpnt],False,(0,255,255),2)

            # cv2.imshow("Converted Image2",img_original)
            if self.warpedpic is not None:
                cv2.imshow("Warped pic",self.warpedpic)
            cv2.imshow("Converted Image",np.hstack([img_original,res]))

            cv2.waitKey(500)

        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

    def waypntcb(self,data):
        self.raw_waypnts = data
        alist = []
        outln = []
        for i in range(len(data.data)):
            if i < len(data.data)-8:
                alist.append(data.data[i].data)
            else:
                outln.append(data.data[i].data)
        self.waypnts = alist
        self.outline = outln
        # print alist
        lvl = 10
        cnt = 0
        change = False
        alt_cnt = 0
        for i in range(25):
            if i == 0:
                name = 'A'
            elif i == 1:
                name = 'B'
            elif i == 2:
                name = 'C'
            elif i == 3:
                name = 'D'
            elif i == 4:
                name = 'E'
            elif i == 5:
                name = 'F'
            elif i == 6:
                name = 'G'
            elif i == 7:
                name = 'H'
            elif i == 8:
                name = 'I'
            elif i == 9:
                name = 'J'
            elif i == 10:
                name = 'K'
            elif i == 11:
                name = 'L'
            elif i == 12:
                name = 'M'
            elif i == 13:
                name = 'N'
            elif i == 14:
                name = 'O'
            elif i == 15:
                name = 'P'
            elif i == 16:
                name = 'Q'
            elif i == 17:
                name = 'R'
            elif i == 18:
                name = 'S'
            elif i == 19:
                name = 'T'
            elif i == 20:
                name = 'U'
            elif i == 21:
                name = 'V'
            elif i == 22:
                name = 'W'
            elif i == 23:
                name = 'X'
            else:
                name = 'Y'

            for j in range(1,lvl):
                cell = name + str(j)
                self.waypnt_dict[cell] = alist[cnt]
                # print cnt
                cnt+=1

            if lvl == 18:
                 change = True
            if lvl < 18 and change == False:
                lvl += 1
            else:
                lvl -= 1
                if alt_cnt < 4:
                    change = False
                    alt_cnt+=1

        self.prey_pathpnt = []
        for i in range(len(self.prey_path)):
            self.prey_pathpnt.append(self.waypnt_dict[self.prey_path[i]])
        self.prey_pathpnt = np.array(self.prey_pathpnt)

        self.predator_pathpnt = []
        for i in range(len(self.predator_path)):
            self.predator_pathpnt.append(self.waypnt_dict[self.predator_path[i]])
        self.predator_pathpnt = np.array(self.predator_pathpnt)
        # print self.waypnt_dict

def nothing(x):
    pass

def main():
    rospy.init_node('sphero_finder', anonymous=False)
    ic = sphero_finder()
    rospy.sleep(1)

    r,g,b = rospy.get_param('prey/prey_mini_setup/color',[0,255,0])
    ic.prey_color_pub.publish(ColorRGBA(r,g,b,1))
    r,g,b = rospy.get_param('predator/predator_mini_setup/color',[255,0,255])
    ic.predator_color_pub.publish(ColorRGBA(r,g,b,1))
    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path("maze_control"), "src", "path.csv")) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            ic.prey_path.append(row[0])
            ic.predator_path.append(row[1])

    with open(os.path.join(rospack.get_path("maze_control"), "src", "waypoints.csv")) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        allpts = []
        for row in readCSV:
            allpts.append(IntList(eval(row[0])))

    alist = Waypoints()
    alist.data = allpts
    ic.waypntcb(alist)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
