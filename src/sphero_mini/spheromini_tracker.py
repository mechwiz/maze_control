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
        self.prey_pathpnt = []
        self.predator_pathpnt = []
        self.predator_path = []
        self.prey_sample = []
        self.predator_sample = []
        self.prey_center = []
        self.predator_center = []
        self.outline = []
        self.warpedpic = None
        self.warped_prey = []
        self.warped_predator = []
        self.obstacles = []
        self.theta = 0
        self.calib = False
        self.hex_dict = {}
        self.start_track = False
        self.bridge = CvBridge()
        self.lower_green = np.array(rospy.get_param('spheromini_tracker/lower_prey'))
        self.upper_green = np.array(rospy.get_param('spheromini_tracker/upper_prey'))
        self.lower_red = np.array(rospy.get_param('spheromini_tracker/lower_predator'))
        self.upper_red = np.array(rospy.get_param('spheromini_tracker/upper_predator'))
        self.radius = rospy.get_param('detect_obstacles/radius',17)
        self.image_sub = rospy.Subscriber("/combined_image",Image,self.imagecb)
        self.image_sub2 = rospy.Subscriber("/warped_image",Image,self.warpedcb)
        self.waypnts_sub = rospy.Subscriber("/waypoints",Waypoints,self.waypntcb)
        self.predator_path_sub = rospy.Subscriber("/predator/path",Waypoints,self.predatorpathcb)
        self.list_pub = rospy.Service("waypoints_fixed",waypoint,self.waypnt_srv)
        self.image_pub = rospy.Publisher("center_point1",Point,queue_size=1)
        self.image_pub2 = rospy.Publisher("center_point2",Point,queue_size=1)
        self.prey_move = rospy.Publisher("prey/move",Point,queue_size=1)
        self.predator_move = rospy.Publisher("predator/move",Point,queue_size=1)
        self.prey_color_pub = rospy.Publisher("prey/set_color",ColorRGBA,queue_size=1)
        self.predator_color_pub = rospy.Publisher("predator/set_color",ColorRGBA,queue_size=1)

    def predatorpathcb(self,data):
        self.predator_path = []
        for i in range(len(data.data)):
            self.predator_path.append(data.data[i].data)

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
            mask = cv2.inRange(hsv,self.lower_green, self.upper_green)
            mask2 = cv2.inRange(hsv,self.lower_red, self.upper_red)
            # res =cv2.bitwise_and(img_original,img_original,mask= mask2)

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

                                # res = cv2.circle(res,(int(x),int(y)),int(radius),(0,255,0),2)
                                img_original = cv2.circle(img_original,(int(x),int(y)),int(radius),(0,255,0),2)

                              #print center
                                self.image_pub.publish(int(x),int(y),0)
                                self.prey_center = [int(x),int(y)]
                                break
                elif len(self.warped_prey)>0:
                    img_original = cv2.circle(img_original,(self.warped_prey[0],self.warped_prey[1]),5,(0,255,0),2)
                    self.image_pub.publish(self.warped_prey[0],self.warped_prey[1],0)
                    self.prey_center = [self.warped_prey[0],self.warped_prey[1]]

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
                                self.predator_center = [int(x),int(y)]
                                break

                elif len(self.warped_predator)>0:
                        img_original = cv2.circle(img_original,(self.warped_predator[0],self.warped_predator[1]),5,(255,0,0),2)
                        self.image_pub2.publish(self.warped_predator[0],self.warped_predator[1],0)
                        self.predator_center = [self.warped_predator[0],self.warped_predator[1]]

            if len(self.prey_center)>0 and len(self.predator_center)>0 and len(self.waypnts)>0:
                prey_idx = closest_node(self.prey_center,self.waypnt_vals)
                predator_idx = closest_node(self.predator_center,self.waypnt_vals)
                self.prey_sample.append(prey_idx)
                self.predator_sample.append(predator_idx)

                if len(self.prey_sample) == 5:
                    prey_idx = stats.mode(self.prey_sample)[0][0]
                    predator_idx = stats.mode(self.predator_sample)[0][0]
                    self.prey_sample = []
                    self.predator_sample = []
                    wp_prey = self.waypnt_vals[prey_idx]
                    wp_predator = self.waypnt_vals[predator_idx]
                    self.prey_move.publish(wp_prey[0],wp_prey[1],0)
                    self.predator_move.publish(wp_predator[0],wp_predator[1],0)

                    if self.start_track == True:
                        if len(self.prey_pathpnt) == 0:
                            self.prey_pathpnt.append(wp_prey)
                            self.predator_pathpnt.append(wp_predator)

                            if os.path.exists(os.path.join(self.rospack.get_path("maze_control"), "src", "observed_path.csv")):
                                os.remove(os.path.join(self.rospack.get_path("maze_control"), "src", "observed_path.csv"))
                            with open(os.path.join(self.rospack.get_path("maze_control"), "src", "observed_path.csv"),mode='a') as csvfile:
                                writeCSV = csv.writer(csvfile, delimiter=',')
                                writeCSV.writerow([self.waypnt_keys[prey_idx],self.waypnt_keys[predator_idx]])

                        else:
                            if wp_prey == self.prey_pathpnt[-1] and wp_predator == self.predator_pathpnt[-1]:
                                pass
                            else:
                                self.prey_pathpnt.append(wp_prey)
                                self.predator_pathpnt.append(wp_predator)
                                with open(os.path.join(self.rospack.get_path("maze_control"), "src", "observed_path.csv"),mode='a') as csvfile:
                                    writeCSV = csv.writer(csvfile, delimiter=',')
                                    writeCSV.writerow([self.waypnt_keys[prey_idx],self.waypnt_keys[predator_idx]])

            # r = 17

            if len(self.waypnts) > 0 and len(self.obstacles) > 0 and self.calib == False:
                xg1,yg1 = self.waypnt_dict['M1']
                xg2,yg2 = self.waypnt_dict['M17']
                pnt_list = []
                x,y = self.waypnt_dict[self.obstacles[0]]
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

                for obst in self.obstacles:
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

            if len(self.prey_pathpnt) > 1:
                cv2.polylines(img_original,[np.array(self.prey_pathpnt,np.int32)],False,(255,255,0),2)

            if len(self.predator_pathpnt) > 1:
                cv2.polylines(img_original,[np.array(self.predator_pathpnt,np.int32)],False,(0,255,255),2)

            if len(self.predator_path) > 0:
                cv2.polylines(img_original,[np.array(self.predator_path,np.int32)],False,(255,0,255),2)

            if len(self.obstacles) > 0:
                for key in self.hex_dict:
                    cv2.polylines(img_original,[np.array(self.hex_dict[key],np.int32)],True,(255,0,0),2)

            cv2.imshow("Converted Image2",img_original)
            if self.warpedpic is not None:
                cv2.imshow("Warped pic",self.warpedpic)
            # cv2.imshow("Converted Image",np.hstack([img_original,res]))

            k = cv2.waitKey(3)
            if k == ord('c'):
                self.start_track = True

        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

    def waypntcb(self,data):
        self.calib = False
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

    # r,g,b = rospy.get_param('prey/prey_mini_setup/color',[0,255,0])
    # ic.prey_color_pub.publish(ColorRGBA(r,g,b,1))
    r,g,b = rospy.get_param('predator/predator_mini_setup/color',[255,0,255])
    ic.predator_color_pub.publish(ColorRGBA(r,g,b,1))

    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path("maze_control"), "src", "obstacles.csv")) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            if len(row) == 0:
                break
            else:
                ic.obstacles.append(row[0])


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
