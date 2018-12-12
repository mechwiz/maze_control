#!/usr/bin/env python
import rospy
import numpy as np
import math
import time

from maze_control.msg import IntList, Waypoints
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32, Int16
from maze_control.srv import offset


class predator_calibrate:

    def __init__(self):
        self.predator_time = time.time()
        self.waypnts = []
        self.waypnt_dict = {}
        self.timer = 0
        self.x_list = []
        self.y_list = []
        self.predator_offset = 0
        self.calibrated = False
        self.waypnt_sub = rospy.Subscriber("/waypoints_fixed",Waypoints,self.waypntcb)
        self.predator_sub = rospy.Subscriber("/center_point2",Point,self.predator_cb)
        self.predator_vel_pub = rospy.Publisher("predator/cmd_vel",Int16,queue_size=1)
        self.predator_heading_pub = rospy.Publisher("predator/set_heading",Int16,queue_size=1)
        self.predator_offset_srv = rospy.Service("predator/offset",offset,self.offset_srv)

    def offset_srv(self,req):
        while self.calibrated == False:
            pass

        return Float32(self.predator_offset)

    def predator_cb(self,data):

        if self.calibrated == False and len(self.waypnt_dict)>0 and time.time()-self.predator_time>0.07:
            self.predator_time = time.time()
            t = rospy.Time.now()
            time_diff = t-self.timer
            ts = time_diff.to_sec()
            # print ts
            if ts < 3:
                self.roll_sphero('Predator',80,0,0)
                if ts > 1:
                    y = data.y
                    x = data.x
                    self.x_list.append(x)
                    self.y_list.append(y)
            else:
                self.roll_sphero('Predator',0,0,0)
                xg1,yg1 = self.waypnt_dict['M1']
                xg2,yg2 = self.waypnt_dict['M17']

                mg = np.polyfit([xg1,xg2],[yg1,yg2],1)[0]
                ml = np.polyfit(self.x_list,self.y_list,1)[0]

                xl1, yl1 = self.x_list[0], self.y_list[0]
                xl2, yl2 = self.x_list[-1],self.y_list[-1]

                v0 = np.array([xg2,yg1]-np.array([xg1,yg2]))
                v1 = np.array([xl2,yl1]-np.array([xl1,yl2]))

                angle = math.degrees(np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1)))
                if angle < 0:
                    angle+=360
                self.predator_offset = angle
                self.calibrated = True

        else:
            pass

    def roll_sphero(self,sph,speed,angle,offset):

        self.predator_vel_pub.publish(Int16(int(speed)))
        self.predator_heading_pub.publish(Int16(int(angle)))

    def waypntcb(self,data):
        alist = []
        for i in range(len(data.data)-8):
            alist.append(data.data[i].data)
        self.waypnts = alist

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

        self.timer = rospy.Time.now()

def main():
    rospy.init_node('predator_calibrate', anonymous=False)
    ic = predator_calibrate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
