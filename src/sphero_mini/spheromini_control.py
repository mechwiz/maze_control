#!/usr/bin/env python
import rospy
import os, rospkg
import cv2
import numpy as np
import csv
import math
import time

from maze_control.msg import IntList, Waypoints
from maze_control.srv import offset
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import ColorRGBA, Float32, Int16
from nav_msgs.msg import Odometry
from pidController2 import pidController



class sphero_control:

    def __init__(self):
        self.prey_time = time.time()
        self.predator_time = time.time()
        self.prey_length = 0
        self.predator_length = 0
        self.prey_percent = 0
        self.predator_percent = 0
        self.prey_error = 0
        self.predator_error = 0
        self.kp = 1
        self.prey_limit = True
        self.predator_limit = True
        self.prey_achieved = []
        self.predator_achieved = []
        self.prey_check = False
        self.predator_check = False
        self.waypnts = []
        self.waypnt_dict = {}
        self.prey_path = []
        self.prey_pathpnt =[]
        self.predator_path = []
        self.predator_pathpnt =[]
        self.prey_speed = 0
        self.predator_speed = 0
        self.prey_offset = 0
        self.predator_offset = 0
        self.prey_cntrl = pidController()
        self.predator_cntrl = pidController()
        self.Kp = rospy.get_param('spheromini_control/Kp')
        self.Ki = rospy.get_param('spheromini_control/Ki')
        self.Kd = rospy.get_param('spheromini_control/Kd')
        self.stopRadius = rospy.get_param('spheromini_control/stopRadius')
        self.maxSpeed = rospy.get_param('spheromini_control/maxSpeed')
        self.minSpeed = rospy.get_param('spheromini_control/minSpeed')
        self.resumeSpeed = rospy.get_param('spheromini_control/resumeSpeed')
        self.waypnt_sub = rospy.Subscriber("/waypoints_fixed",Waypoints,self.waypntcb)
        self.prey_sub = rospy.Subscriber("/center_point1",Point,self.prey_cb)
        self.predator_sub = rospy.Subscriber("/center_point2",Point,self.predator_cb)
        self.prey_vel_pub = rospy.Publisher("prey/cmd_vel",Int16,queue_size=1)
        self.predator_vel_pub = rospy.Publisher("predator/cmd_vel",Int16,queue_size=1)
        self.prey_heading_pub = rospy.Publisher("prey/set_heading",Int16,queue_size=1)
        self.predator_heading_pub = rospy.Publisher("predator/set_heading",Int16,queue_size=1)


    def prey_cb(self,data):
        if len(self.prey_pathpnt) > 0 and len(self.prey_achieved) < len(self.prey_path) and np.abs(self.prey_offset) > 0 and time.time()-self.prey_time > 0.08:
            self.prey_time = time.time()
            y = data.y
            x = data.x

            targetnum = len(self.prey_achieved)
            xt,yt = self.prey_pathpnt[targetnum]
            self.prey_percent = self.get_precentTraj(targetnum,'prey',[xt,yt],[x,y])

            angle, distance = vector_to_target(x,yt,xt,y)
            outspeed = self.prey_cntrl.getPIDSpeed(distance,self.Kp,self.Ki,self.Kd,self.stopRadius,self.maxSpeed,self.minSpeed,self.resumeSpeed) - self.prey_error
            outspeed = max(outspeed,0)

            if distance < 20:
                self.prey_achieved.append(self.prey_pathpnt[targetnum])
                targetnum = len(self.prey_achieved)
                if len(self.prey_achieved) < len(self.prey_path):
                    xt,yt = self.prey_pathpnt[targetnum]
                    angle, distance = vector_to_target(x,yt,xt,y)
                    self.prey_cntrl.reset()
                    outspeed = self.prey_cntrl.getPIDSpeed(distance,self.Kp,self.Ki,self.Kd,self.stopRadius,self.maxSpeed,self.minSpeed,self.resumeSpeed) - self.prey_error
                    outspeed = max(outspeed,0)
                    self.roll_sphero('Prey',outspeed,angle,self.prey_offset)
                else:
                    self.roll_sphero('Prey',0,angle,self.prey_offset)

            else:
                self.roll_sphero('Prey',outspeed,angle,self.prey_offset)


        if self.prey_percent > 0 and self.predator_percent > 0:
            error = self.predator_percent - self.prey_percent

            if error > 0:
                self.predator_error = 0
                self.prey_error = self.kp*abs(error)
            else:
                self.prey_error = 0
                self.predator_error = self.kp*abs(error)

    def predator_cb(self,data):
        if len(self.predator_pathpnt) > 0 and len(self.predator_achieved) < len(self.predator_path) and np.abs(self.predator_offset) > 0 and time.time()-self.predator_time > 0.08:
            self.predator_time = time.time()
            y = data.y
            x = data.x

            targetnum = len(self.predator_achieved)
            xt,yt = self.predator_pathpnt[targetnum]
            self.predator_percent = self.get_precentTraj(targetnum,'predator',[xt,yt],[x,y])

            angle, distance = vector_to_target(x,yt,xt,y)
            outspeed = self.predator_cntrl.getPIDSpeed(distance,self.Kp,self.Ki,self.Kd,self.stopRadius,self.maxSpeed,self.minSpeed,self.resumeSpeed) - self.predator_error
            outspeed = max(outspeed,0)

            if distance < 20:
                self.predator_achieved.append(self.predator_pathpnt[targetnum])
                targetnum = len(self.predator_achieved)
                if len(self.predator_achieved) < len(self.predator_path):
                    xt,yt = self.predator_pathpnt[targetnum]
                    angle, distance = vector_to_target(x,yt,xt,y)
                    self.predator_cntrl.reset()
                    outspeed = self.predator_cntrl.getPIDSpeed(distance,self.Kp,self.Ki,self.Kd,self.stopRadius,self.maxSpeed,self.minSpeed,self.resumeSpeed) - self.predator_error
                    outspeed = max(outspeed,0)
                    self.roll_sphero('Predator',outspeed,angle,self.predator_offset)
                else:
                    self.roll_sphero('Predator',0,angle,self.predator_offset)

            else:
                self.roll_sphero('Predator',outspeed,angle,self.predator_offset)

    def roll_sphero(self,sph,speed,angle,offset):

        angle = offset - angle

        if sph == 'Prey':
            self.prey_vel_pub.publish(Int16(int(speed)))
            self.prey_heading_pub.publish(Int16(int(angle)))

        else:
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

        # print self.waypnt_dict
        self.prey_pathpnt = []
        for i in range(len(self.prey_path)):
            self.prey_pathpnt.append(self.waypnt_dict[self.prey_path[i]])

        self.predator_pathpnt = []
        for i in range(len(self.predator_path)):
            self.predator_pathpnt.append(self.waypnt_dict[self.predator_path[i]])

        for i in range(1,len(self.prey_pathpnt)):
            x1,y1 = self.prey_pathpnt[i-1]
            x2,y2 = self.prey_pathpnt[i]
            angle,distance = vector_to_target(x1,y1,x2,y2)
            self.prey_length+=distance
            x1,y1 = self.predator_pathpnt[i-1]
            x2,y2 = self.predator_pathpnt[i]
            angle,distance = vector_to_target(x1,y1,x2,y2)
            self.predator_length+=distance

    def get_precentTraj(self,num_achieved,animal,target,current):
        length_tot = 0
        xc,yc = current
        xt,yt = target

        if animal == 'prey':
            pathpnt = self.prey_pathpnt
            total = self.prey_length
        else:
            pathpnt = self.predator_pathpnt
            total = self.predator_length

        for i in range(num_achieved,len(self.prey_pathpnt)):
            if i == num_achieved:
                x1,y1 = xc,yc
            else:
                x1,y1 = pathpnt[i-1]
            x2,y2 = pathpnt[i]
            angle,distance = vector_to_target(x1,y1,x2,y2)
            length_tot+=distance

        return length_tot/total * 100.0

def vector_to_target(currentX, currentY, targetX, targetY):
    '''
        Returns distance and angle between two points (in degrees)
    '''
    deltaX = targetX - currentX
    deltaY = targetY - currentY
    angle = math.degrees(math.atan2(deltaY, deltaX))
    distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

    return angle, distance

def main():
    getOffset = rospy.ServiceProxy("prey/offset",offset)
    rospy.wait_for_service("prey/offset")
    try:
        prey_offset = getOffset()

        pass
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    getOffset = rospy.ServiceProxy("predator/offset",offset)
    rospy.wait_for_service("predator/offset")
    try:
        predator_offset = getOffset()

        pass
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.init_node('sphero_control', anonymous=False)
    ic = sphero_control()

    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path("maze_control"), "src", "path.csv")) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            ic.prey_path.append(row[0])
            ic.predator_path.append(row[1])
    ic.prey_offset = prey_offset.offset.data
    ic.predator_offset = predator_offset.offset.data

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
