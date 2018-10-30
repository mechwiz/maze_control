#!/usr/bin/env python
import rospy
import numpy as np
import math

from maze_control.msg import IntList, Waypoints
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32
from maze_control.srv import offset


class prey_calibrate:

    def __init__(self):
        self.waypnts = []
        self.waypnt_dict = {}
        self.timer = 0
        self.x_list = []
        self.y_list = []
        self.prey_offset = 0
        self.calibrated = False
        self.waypnt_sub = rospy.Subscriber("/waypoints_fixed",Waypoints,self.waypntcb)
        self.prey_sub = rospy.Subscriber("/center_point1",Point,self.prey_cb)
        self.prey_vel_pub = rospy.Publisher("prey/cmd_vel",Twist,queue_size=10)
        self.prey_offset_srv = rospy.Service("prey/offset",offset,self.offset_srv)

    def offset_srv(self,req):
        while self.calibrated == False:
            pass

        return Float32(self.prey_offset)

    def prey_cb(self,data):

        if self.calibrated == False and len(self.waypnt_dict)>0:
            t = rospy.Time.now()
            time_diff = t-self.timer
            ts = time_diff.to_sec()
            # print ts
            if ts < 3:
                self.roll_sphero('Prey',30,0,0)
                if ts > 1:
                    y = data.y
                    x = data.x
                    self.x_list.append(x)
                    self.y_list.append(y)
            else:
                self.roll_sphero('Prey',0,0,0)
                xg1,yg1 = self.waypnt_dict['E1']
                xg2,yg2 = self.waypnt_dict['E9']

                mg = np.polyfit([xg1,xg2],[yg1,yg2],1)[0]
                ml = np.polyfit(self.x_list,self.y_list,1)[0]

                xl1, yl1 = self.x_list[0], self.y_list[0]
                xl2, yl2 = self.x_list[-1],self.y_list[-1]

                v0 = np.array([xg2,yg1]-np.array([xg1,yg2]))
                v1 = np.array([xl2,yl1]-np.array([xl1,yl2]))

                angle = math.degrees(np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1)))
                self.prey_offset = angle
                self.calibrated = True

        else:
            pass

    def roll_sphero(self,sph,speed,angle,offset):
        newTwist = Twist()
        newTwist.linear.x = speed

        if sph == 'Prey':
            self.prey_vel_pub.publish(newTwist)

    def waypntcb(self,data):
        alist = []
        for i in range(len(data.data)-6):
            alist.append(data.data[i].data)
        self.waypnts = alist

        lvl = 6
        cnt = 0
        change = False
        for i in range(9):
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
            else:
                name = 'I'

            for j in range(1,lvl):
                cell = name + str(j)
                self.waypnt_dict[cell] = alist[cnt]
                # print cnt
                cnt+=1

            if lvl == 10:
                 change = True
            if lvl < 10 and change == False:
                lvl += 1
            else:
                lvl -= 1
        self.timer = rospy.Time.now()

def main():
    rospy.init_node('prey_calibrate', anonymous=True)
    ic = prey_calibrate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
