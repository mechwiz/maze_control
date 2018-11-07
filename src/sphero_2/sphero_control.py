#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import csv
import math

from maze_control.msg import IntList, Waypoints
from maze_control.srv import offset
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import ColorRGBA, Float32
from nav_msgs.msg import Odometry
from pidController import pidController



class sphero_control:

    def __init__(self):
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
        self.prey = pidController()
        self.predator = pidController()
        self.waypnt_sub = rospy.Subscriber("/waypoints_fixed",Waypoints,self.waypntcb)
        self.prey_sub = rospy.Subscriber("/center_point1",Point,self.prey_cb)
        self.predator_sub = rospy.Subscriber("/center_point2",Point,self.predator_cb)
        self.prey_odm_sub = rospy.Subscriber("/prey/odom",Odometry,self.prey_odom)
        self.prey_vel_pub = rospy.Publisher("prey/cmd_vel",Twist,queue_size=1)
        self.predator_odm_sub = rospy.Subscriber("/predator/odom",Odometry,self.predator_odom)
        self.predator_vel_pub = rospy.Publisher("predator/cmd_vel",Twist,queue_size=1)
        # self.prey_heading_pub = rospy.Publisher("prey/set_heading",Float32,queue_size=10)

    # def prey_offset_cb(self,data):
    #     self.prey_offset = data.data

    def prey_odom(self,data):
        self.prey_speed = math.sqrt((data.twist.twist.linear.x*100)**2 +(data.twist.twist.linear.y*100)**2)
        # print self.prey_speed
    def prey_cb(self,data):
        # print self.prey_offset
        if len(self.prey_pathpnt) > 0 and len(self.prey_achieved) < len(self.prey_path) and np.abs(self.prey_offset) > 0:
            y = data.y
            x = data.x

            targetnum = len(self.prey_achieved)
            # print targetnum
            xt,yt = self.prey_pathpnt[targetnum]

            angle, distance = vector_to_target(x,y,xt,yt)
            outspeed = self.prey.getPIDSpeed(distance,self.prey_speed)
            # print angle, distance

            # if distance < 20:
            #     self.prey_achieved.append(self.prey_pathpnt[targetnum])
            #     targetnum = len(self.prey_achieved)
            #     if len(self.prey_achieved) < len(self.prey_path):
            #         # self.roll_sphero('Prey',outspeed,-(angle+180),self.prey_offset)
            #         xt,yt = self.prey_pathpnt[targetnum]
            #         angle, distance = vector_to_target(x,y,xt,yt)
            #         self.prey.reset()
            #         outspeed = self.prey.getPIDSpeed(distance,self.prey_speed)
            #         self.roll_sphero('Prey',outspeed,-angle,-self.prey_offset)
            #         rospy.sleep(0.4)

            if distance < 20 and self.prey_limit == True:
                prev_angle = angle
                if targetnum+1 != len(self.prey_path):
                    xt,yt = self.prey_pathpnt[targetnum+1]
                    angle, distance = vector_to_target(x,y,xt,yt)
                    if abs(angle-prev_angle) > 60 and self.prey_check == False:
                        self.roll_sphero('Prey',-outspeed,-prev_angle,-self.prey_offset)
                        rospy.sleep(0.3)
                        self.prey_check = True
                        return

                self.prey_achieved.append(self.prey_pathpnt[targetnum])
                targetnum = len(self.prey_achieved)

                if targetnum == len(self.prey_pathpnt):
                    self.roll_sphero('Prey',-outspeed,-prev_angle,-self.prey_offset)
                    return

                xt,yt = self.prey_pathpnt[targetnum]
                angle, distance = vector_to_target(x,y,xt,yt)
                # print prev_angle,angle
                self.prey.reset()
                outspeed = self.prey.getPIDSpeed(distance,self.prey_speed)
                self.roll_sphero('Prey',outspeed,-angle,-self.prey_offset)
                rospy.sleep(0.4)
                self.prey_check = False

            # print outspeed
            else:
                # print distance,outspeed
                self.roll_sphero('Prey',outspeed,-angle,-self.prey_offset)
                # rospy.sleep(0.01)
            # rospy.sleep(0.5)
        if len(self.prey_achieved)-len(self.predator_achieved)>0:
            self.prey_limit = False
            self.predator_limit = True
        elif len(self.predator_achieved)-len(self.prey_achieved)>0:
            self.prey_limit = True
            self.predator_limit = False
        else:
            self.prey_limit = True
            self.predator_limit = True


    def predator_odom(self,data):
        self.predator_speed = math.sqrt((data.twist.twist.linear.x*100)**2 +(data.twist.twist.linear.y*100)**2)
        # print self.prey_speed

    def predator_cb(self,data):
        # print self.prey_offset
        if len(self.predator_pathpnt) > 0 and len(self.predator_achieved) < len(self.predator_path) and np.abs(self.predator_offset) > 0:
            y = data.y
            x = data.x

            targetnum = len(self.predator_achieved)
            # print targetnum
            xt,yt = self.predator_pathpnt[targetnum]

            angle, distance = vector_to_target(x,y,xt,yt)
            outspeed = self.predator.getPIDSpeed(distance,self.predator_speed)
            # print angle

            # if distance < 20:
            #     self.predator_achieved.append(self.predator_pathpnt[targetnum])
            #     targetnum = len(self.predator_achieved)
            #     if len(self.predator_achieved) < len(self.predator_path):
            #         # self.roll_sphero('predator',outspeed,-(angle+180),self.predator_offset)
            #         xt,yt = self.predator_pathpnt[targetnum]
            #         angle, distance = vector_to_target(x,y,xt,yt)
            #         self.predator.reset()
            #         outspeed = self.predator.getPIDSpeed(distance,self.predator_speed)
            #         self.roll_sphero('Predator',outspeed,-angle,-self.predator_offset)
            #         rospy.sleep(0.4)

            if distance < 20 and self.predator_limit == True:
                prev_angle = angle
                if targetnum+1 != len(self.predator_path):
                    xt,yt = self.predator_pathpnt[targetnum+1]
                    angle, distance = vector_to_target(x,y,xt,yt)
                    if abs(angle-prev_angle) > 60 and self.predator_check == False:
                        self.roll_sphero('Predator',-outspeed,-prev_angle,-self.predator_offset)
                        rospy.sleep(0.3)
                        self.predator_check = True
                        return

                self.predator_achieved.append(self.predator_pathpnt[targetnum])
                targetnum = len(self.predator_achieved)

                if targetnum == len(self.predator_pathpnt):
                    self.roll_sphero('Predator',-outspeed,-prev_angle,-self.predator_offset)
                    return

                xt,yt = self.predator_pathpnt[targetnum]
                angle, distance = vector_to_target(x,y,xt,yt)
                # print prev_angle,angle
                self.predator.reset()
                outspeed = self.predator.getPIDSpeed(distance,self.predator_speed)
                self.roll_sphero('Predator',outspeed,-angle,-self.predator_offset)
                rospy.sleep(0.4)
                self.predator_check = False
            else:
                self.roll_sphero('Predator',outspeed,-angle,-self.predator_offset)
                # rospy.sleep(0.01)

    def roll_sphero(self,sph,speed,angle,offset):
        newTwist = Twist()
        # newTwist.linear.y = speed
        angle = angle + offset
        newTwist.linear.x = math.cos(math.radians(angle))*speed
        newTwist.linear.y = math.sin(math.radians(angle))*speed

        if sph == 'Prey':
            self.prey_vel_pub.publish(newTwist)
        else:
            self.predator_vel_pub.publish(newTwist)
            # self.prey_heading_pub.publish(Float32(angle+offset))

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

        # print self.waypnt_dict
        self.prey_pathpnt = []
        for i in range(len(self.prey_path)):
            self.prey_pathpnt.append(self.waypnt_dict[self.prey_path[i]])

        self.predator_pathpnt = []
        for i in range(len(self.predator_path)):
            self.predator_pathpnt.append(self.waypnt_dict[self.predator_path[i]])

def closest_node(node, nodes):
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

    rospy.init_node('sphero_control', anonymous=True)
    ic = sphero_control()

    with open('/home/mikewiz/project_ws/src/maze_control/src/path.csv') as csvfile:
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