#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import csv

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from maze_control.msg import IntList, Waypoints
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
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imagecb)
        self.image_sub = rospy.Subscriber("/waypoints",Waypoints,self.waypntcb)
        self.list_pub = rospy.Publisher("waypoints_fixed",Waypoints,queue_size=10)
        self.image_pub = rospy.Publisher("center_point1",Point,queue_size=1)
        self.image_pub2 = rospy.Publisher("center_point2",Point,queue_size=1)
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
            # lower_red = np.array([140,75,150])
            # upper_red = np.array([180,170,255])
            # lower_red = np.array([140,10,0])
            # upper_red = np.array([180,170,255])
            # lower_red2 = np.array([70,90,175])
            # upper_red2 = np.array([130,190,255])
            # lower_red2 = np.array([70,90,175])
            # upper_red2 = np.array([120,217,255])
            # lower_red2 = np.array([0,180,125])
            # upper_red2 = np.array([10,255,255])
            lower_red2 = np.array([100,25,200])
            upper_red2 = np.array([164,255,255])

            # lower_red = np.array([140,10,150])
            # upper_red = np.array([180,170,255])

            # lower_red = np.array([45,10,150])
            # upper_red = np.array([100,170,255])
            lower_red = np.array([80,30,220])
            upper_red = np.array([100,255,255])

            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # img_original = cv2.flip(img_original,1)
            hsv = cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,lower_red, upper_red)
            mask2 = cv2.inRange(hsv,lower_red2, upper_red2)
            res =cv2.bitwise_and(img_original,img_original,mask= mask)

            contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            contour2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            if len(contour) > 0 and len(self.waypnts)>0:
                for c in contour:
                    ((x,y),radius) = cv2.minEnclosingCircle(c)
                    if cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False)>0:
                        M = cv2.moments(c)

                        # print radius, cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False), (x,y)
                        if radius > 3:
                            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        # else:
                        #   center = (0,0)
                            # res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),(0,255,0),2)
                            img_original = cv2.circle(img_original,(int(center[0]),int(center[1])),int(radius),(0,255,0),2)

                          #print center
                            self.image_pub.publish(center[0],center[1],0)
                            break

            if len(contour2) > 0 and len(self.waypnts)>0:
                for c in contour2:
                    ((x,y),radius) = cv2.minEnclosingCircle(c)
                    if cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False)>0:
                        M = cv2.moments(c)

                # c = max(contour2, key = cv2.contourArea)
                # ((x,y),radius) = cv2.minEnclosingCircle(c)
                # M = cv2.moments(c)
                # print radius, cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),True)
                        if radius > 3:
                            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        # else:
                        #   center = (0,0)
                            # res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),(255,0,0),2)
                            img_original = cv2.circle(img_original,(int(center[0]),int(center[1])),int(radius),(255,0,0),2)

                          #print center
                            self.image_pub2.publish(center[0],center[1],0)
                            break

            if len(self.waypnts) > 0:
                for wp in self.waypnts:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),-1)

                pts = np.array(self.outline,np.int32)
                # pts = pts.reshape((-1,1,2))
                cv2.polylines(img_original,[pts],True,(0,255,0),3)
                # cv2.drawContours(img_original, np.array(self.outline), -1, (0, 255, 0), 3)

                for wp in self.outline:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(255,0,0),-1)

            if len(self.prey_pathpnt) > 0:
                cv2.polylines(img_original,[self.prey_pathpnt],False,(255,255,0),2)

            if len(self.predator_pathpnt) > 0:
                cv2.polylines(img_original,[self.predator_pathpnt],False,(0,255,255),2)

            cv2.imshow("Converted Image",img_original)
            # cv2.imshow("Converted Image",np.hstack([img_original,res]))

            k = cv2.waitKey(3)
            if k == ord('f') and len(self.waypnts)>0:
                self.list_pub.publish(self.raw_waypnts)
                rospy.loginfo('Points Captured')
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

    ic.prey_color_pub.publish(ColorRGBA(0,255,0,1))
    ic.predator_color_pub.publish(ColorRGBA(255,0,255,1))
    with open('/home/mikewiz/project_ws/src/maze_control/src/path.csv') as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            ic.prey_path.append(row[0])
            ic.predator_path.append(row[1])
    # print ic.path
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
