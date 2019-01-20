#!/usr/bin/env python
import rospy
import os, rospkg
import cv2
import numpy as np
import csv
import math

from cv_bridge import CvBridge, CvBridgeError
from scipy import stats
from sensor_msgs.msg import Image
from maze_control.srv import waypoint
from maze_control.msg import IntList, Waypoints
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class sphero_tracker:

    def __init__(self):
        self.raw_waypnts = Waypoints()
        self.rospack = rospkg.RosPack()
        self.waypnts = []
        self.waypnt_dict = {}
        self.waypnt_keys = []
        self.waypnt_vals = []
        self.outline = []
        self.obstacle_list = []
        self.obstacle_dict = {}
        self.cell_order = []
        self.hex_dict = {}
        self.calib = False
        self.theta = 0

        self.bridge = CvBridge()
        self.lower_obst = np.array(rospy.get_param('detect_obstacles/lower_obst'))
        self.upper_obst = np.array(rospy.get_param('detect_obstacles/upper_obst'))
        self.radius = rospy.get_param('detect_obstacles/radius')
        self.image_sub = rospy.Subscriber("/combined_image",Image,self.imagecb)
        self.waypnts_sub = rospy.Subscriber("/waypoints",Waypoints,self.waypntcb)
        # self.list_pub = rospy.Service("waypoints_fixed",waypoint,self.waypnt_srv)



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

            # lower_green = np.array([1,125,255])
            # upper_green = np.array([100,255,255])
            # lower_red = np.array([133,30,250])
            # upper_red = np.array([153,210,255])

            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,self.lower_obst, self.upper_obst)
            # res =cv2.bitwise_and(img_original,img_original,mask= mask2)

            contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if len(self.waypnts)>0:
                if len(contour) > 0:
                    for c in contour:
                        ((x,y),radius) = cv2.minEnclosingCircle(c)
                        if cv2.pointPolygonTest(np.array(self.outline),(int(x),int(y)),False)>0:
                            # M = cv2.moments(c)

                            if radius > 5:
                                cv2.drawContours(img_original,[c],0,(0,255,0),2)
                                # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                                # res = cv2.circle(res,(int(x),int(y)),int(radius),(0,255,0),2)

                                for i in range(len(self.waypnt_vals)):
                                    wp = self.waypnt_vals[i]
                                    if cv2.pointPolygonTest(c,(int(wp[0]),int(wp[1])),False)>0:
                                        k = self.waypnt_keys[i]
                                        if k in self.obstacle_dict.keys():
                                            self.obstacle_dict[k] += 1
                                        else:
                                            self.obstacle_dict[k] = 1

            if 100 in self.obstacle_dict.values():
                self.obstacle_list = []
                self.hex_dict = {}
                vals = self.obstacle_dict.values()
                keys = self.obstacle_dict.keys()

                for i in range(len(vals)):
                    if vals[i] > 20:
                        self.obstacle_list.append(keys[i])

                # r = 17

                if self.calib == False:
                    xg1,yg1 = self.waypnt_dict['M1']
                    xg2,yg2 = self.waypnt_dict['M17']
                    pnt_list = []
                    x,y = self.waypnt_dict[self.obstacle_list[0]]
                    for i in range(6):
                        xn = self.radius*np.cos(2*np.pi*i/6.0) + x
                        yn = self.radius*np.sin(2*np.pi*i/6.0) + y
                        pnt_list.append([xn,yn])

                    xl1,yl1 = pnt_list[0]
                    xl2,yl2 = pnt_list[3]

                    v0 = np.array([xg2,yg1]-np.array([xg1,yg2]))
                    v1 = np.array([xl2,yl1]-np.array([xl1,yl2]))

                    angle = math.degrees(np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1)))
                    global_angle,distance = vector_to_target(xg1,yg2,xg2,yg1)
                    angle += global_angle

                    self.theta = math.radians(angle)
                    self.calib = True

                self.hex_dict = {}
                for obst in self.obstacle_list:
                    x,y = self.waypnt_dict[obst]
                    self.hex_dict[obst] = []
                    for i in range(6):
                        xn = self.radius*np.cos(2*np.pi*i/6.0 + self.theta) + x
                        yn = self.radius*np.sin(2*np.pi*i/6.0 + self.theta) + y
                        self.hex_dict[obst].append([xn,yn])


            if len(self.waypnts) > 0:
                for wp in self.waypnts:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),-1)

                pts = np.array(self.outline,np.int32)
                cv2.polylines(img_original,[pts],True,(0,255,0),3)

                for wp in self.outline:
                    cv2.circle(img_original,(wp[0],wp[1]),5,(255,0,0),-1)

            if len(self.obstacle_list) > 0:
                for key in self.hex_dict:
                    # print np.array(self.hex_dict[key],np.int32)
                    cv2.polylines(img_original,[np.array(self.hex_dict[key],np.int32)],True,(255,0,0),2)


            cv2.imshow("Converted Image2",img_original)
            # cv2.imshow("Converted Image",np.hstack([img_original,res]))

            k = cv2.waitKey(3)
            if k == ord('c') and len(self.obstacle_list)>0:
                if os.path.exists(os.path.join(self.rospack.get_path("maze_control"), "src", "obstacles.csv")):
                        os.remove(os.path.join(self.rospack.get_path("maze_control"), "src", "obstacles.csv"))

                for obst in self.cell_order:
                    if obst in self.obstacle_list:
                        with open(os.path.join(self.rospack.get_path("maze_control"), "src", "obstacles.csv"),mode='a') as csvfile:
                            writeCSV = csv.writer(csvfile, delimiter=',')
                            writeCSV.writerow([obst])


        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

    def waypntcb(self,data):
        self.raw_waypnts = data
        self.obstacle_dict = {}
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
        self.cell_order = []
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
                self.cell_order.append(cell)
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

        self.waypnt_keys = self.waypnt_dict.keys()
        self.waypnt_vals = self.waypnt_dict.values()

def closest_node(node, nodes):
    node = np.asarray(node)
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

def vector_to_target(currentX, currentY, targetX, targetY):
    '''
        Returns distance and angle between two points (in degrees)
    '''
    deltaX = targetX - currentX
    deltaY = targetY - currentY
    angle = math.degrees(math.atan2(deltaY, deltaX))
    distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

    return angle, distance

def nothing(x):
    pass

def main():
    rospy.init_node('sphero_tracker', anonymous=False)
    ic = sphero_tracker()
    rospy.sleep(1)
    rospack = rospkg.RosPack()
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
