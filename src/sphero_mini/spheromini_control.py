#!/usr/bin/env python
import rospy
import os, rospkg
import cv2
import numpy as np
import csv
import math
import time

from maze_control.msg import IntList, Waypoints
from maze_control.srv import offset, waypoint
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
        self.prey_boost = 0
        self.predator_boost = 0
        self.prey_distance = []
        self.predator_distance = []
        self.predator_error = 0
        self.kp = rospy.get_param('spheromini_control/kp_track')
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
        self.obstacles = []
        self.obstacle_pnts = []
        self.calib = False
        self.hex_dict = {}
        self.theta = 0
        self.prey_path2 = []
        self.predator_path2 = []
        self.prey_plan = False
        self.predator_plan =False
        self.prey_start = False
        self.predator_start = False
        self.prey_finish = False
        self.predator_finish = False
        self.waypnt_keys = []
        self.waypnt_vals = []
        self.prey_cntrl = pidController()
        self.predator_cntrl = pidController()
        self.Kp = rospy.get_param('spheromini_control/Kp')
        self.Ki = rospy.get_param('spheromini_control/Ki')
        self.Kd = rospy.get_param('spheromini_control/Kd')
        self.stopRadius = rospy.get_param('spheromini_control/stopRadius')
        self.maxSpeed = rospy.get_param('spheromini_control/maxSpeed')
        self.minSpeed = rospy.get_param('spheromini_control/minSpeed')
        self.resumeSpeed = rospy.get_param('spheromini_control/resumeSpeed')
        self.prey_sub = rospy.Subscriber("/center_point1",Point,self.prey_cb)
        self.predator_sub = rospy.Subscriber("/center_point2",Point,self.predator_cb)
        self.prey_vel_pub = rospy.Publisher("prey/cmd_vel",Int16,queue_size=1)
        self.predator_vel_pub = rospy.Publisher("predator/cmd_vel",Int16,queue_size=1)
        self.prey_heading_pub = rospy.Publisher("prey/set_heading",Int16,queue_size=1)
        self.predator_heading_pub = rospy.Publisher("predator/set_heading",Int16,queue_size=1)
        self.prey_path_pub = rospy.Publisher("prey/path",Waypoints,queue_size=1)
        self.predator_path_pub = rospy.Publisher("predator/path",Waypoints,queue_size=1)

    def prey_cb(self,data):
        if self.prey_plan == False and len(self.waypnts)>0:
            pnt = [data.x,data.y]
            pinit = self.waypnts[closest_node(pnt,self.waypnts)]
            pgoal = self.prey_pathpnt[0]
            if pinit == pgoal:
                self.prey_path2 = np.copy(self.prey_pathpnt).tolist()
                self.prey_pathpnt = []
            else:
                pathpnt,path = self.plan_path([pinit[0],pinit[1]],[pgoal[0],pgoal[1]])
                self.prey_path2 = np.copy(self.prey_pathpnt).tolist()
                self.prey_pathpnt = pathpnt
                self.prey_path_pub.publish(path)
            self.prey_plan = True

        if len(self.waypnts)>0 and self.prey_start == False and len(self.prey_achieved) == len(self.prey_pathpnt):
            self.prey_start = True

        if self.prey_start == True and self.predator_start == True and self.prey_finish == False:
            self.prey_pathpnt = self.prey_path2
            self.prey_achieved = []
            self.prey_finish = True

        if  len(self.prey_pathpnt) > 0 and len(self.prey_achieved) < len(self.prey_pathpnt) and np.abs(self.prey_offset) > 0 and time.time()-self.prey_time > 0.08:
            self.prey_time = time.time()
            y = data.y
            x = data.x

            targetnum = len(self.prey_achieved)
            xt,yt = self.prey_pathpnt[targetnum]

            if self.prey_start == True and self.predator_start == True:
                self.prey_percent = self.get_precentTraj(targetnum,'prey',[xt,yt],[x,y])

            angle, distance = vector_to_target(x,yt,xt,y)

            outspeed = self.prey_cntrl.getPIDSpeed(distance,self.Kp,self.Ki,self.Kd,self.stopRadius,self.maxSpeed,self.minSpeed,self.resumeSpeed) - self.prey_error
            outspeed = max(outspeed,0)

            self.prey_distance.append(distance)
            if len(self.prey_distance) == 10:
                if abs(distance-sum(self.prey_distance)/10.0) < 5:
                    self.prey_boost += 10
                    outspeed += self.prey_boost
                    outspeed = min(outspeed, 255)
                else:
                    self.prey_boost = 0
                self.prey_distance.pop(0)

            if distance < 20:
                self.prey_achieved.append(self.prey_pathpnt[targetnum])
                targetnum = len(self.prey_achieved)
                if len(self.prey_achieved) < len(self.prey_pathpnt):
                    self.prey_distance = []
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
        if self.predator_plan == False and len(self.waypnts)>0:
            pnt = [data.x,data.y]
            pinit = self.waypnts[closest_node(pnt,self.waypnts)]
            pgoal = self.predator_pathpnt[0]
            if pinit == pgoal:
                self.predator_path2 = np.copy(self.predator_pathpnt).tolist()
                self.predator_pathpnt = []
            else:
                pathpnt,path = self.plan_path([pinit[0],pinit[1]],[pgoal[0],pgoal[1]])
                self.predator_path2 = np.copy(self.predator_pathpnt).tolist()
                self.predator_pathpnt = pathpnt
                self.predator_path_pub.publish(path)
            self.predator_plan = True

        if len(self.waypnts)>0 and self.predator_start == False and len(self.predator_achieved) == len(self.predator_pathpnt):
            self.predator_start = True

        if self.predator_start == True and self.prey_start == True and self.predator_finish == False:
            self.predator_pathpnt = self.predator_path2
            self.predator_achieved = []
            self.predator_finish = True

        if len(self.predator_pathpnt) > 0 and len(self.predator_achieved) < len(self.predator_pathpnt) and np.abs(self.predator_offset) > 0 and time.time()-self.predator_time > 0.08:
            self.predator_time = time.time()
            y = data.y
            x = data.x

            targetnum = len(self.predator_achieved)
            xt,yt = self.predator_pathpnt[targetnum]

            if self.predator_start == True and self.prey_start == True:
                self.predator_percent = self.get_precentTraj(targetnum,'predator',[xt,yt],[x,y])

            angle, distance = vector_to_target(x,yt,xt,y)
            outspeed = self.predator_cntrl.getPIDSpeed(distance,self.Kp,self.Ki,self.Kd,self.stopRadius,self.maxSpeed,self.minSpeed,self.resumeSpeed) - self.predator_error
            outspeed = max(outspeed,0)

            self.predator_distance.append(distance)
            if len(self.predator_distance) == 10:
                if abs(distance-sum(self.predator_distance)/10.0) < 5:
                    self.predator_boost += 10
                    outspeed += self.predator_boost
                    outspeed = min(outspeed, 255)
                else:
                    self.predator_boost = 0
                self.predator_distance.pop(0)

            if distance < 20:
                self.predator_achieved.append(self.predator_pathpnt[targetnum])
                targetnum = len(self.predator_achieved)
                if len(self.predator_achieved) < len(self.predator_pathpnt):
                    self.predator_distance = []
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
        for i in range(len(data.data.data)-8):
            alist.append(data.data.data[i].data)
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

        self.waypnt_keys = self.waypnt_dict.keys()
        self.waypnt_vals = self.waypnt_dict.values()

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

        if len(self.obstacles) > 0:
            for obst in self.obstacles:
                self.obstacle_pnts.append(self.waypnt_dict[obst])

        r = 17
        if len(self.obstacles) > 0 and self.calib == False:
            xg1,yg1 = self.waypnt_dict['M1']
            xg2,yg2 = self.waypnt_dict['M17']
            pnt_list = []
            x,y = self.waypnt_dict[self.obstacles[0]]
            for i in range(6):
                xn = r*np.cos(2*np.pi*i/6.0) + x
                yn = r*np.sin(2*np.pi*i/6.0) + y
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
                    xn = r*np.cos(2*np.pi*i/6.0 + self.theta) + x
                    yn = r*np.sin(2*np.pi*i/6.0 + self.theta) + y
                    self.hex_dict[obst].append([xn,yn])

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

    def plan_path(self,pinit,pgoal):
        frontier = {}
        nextFrontier = [[-1,-1],pinit]
        frontier[str(nextFrontier)] = getHeuristic(pinit,pgoal)
        E = []
        E.append(nextFrontier)
        checkedFrontiers = []

        while len(frontier) > 0:
            x,y = nextFrontier[1]
            if nextFrontier[1] in checkedFrontiers:
                pass
            else:
                checkedFrontiers.append(nextFrontier[1])
            frontier.pop(str(nextFrontier))

            neighbors = self.findNeighbors([x,y])
            # print '**********'
            # print [x,y],neighbors

            for n in neighbors:
                if n in checkedFrontiers:
                    pass
                else:
                    nextFrontier = [[x,y],[n[0],n[1]]]
                    E.append(nextFrontier)
                    frontier[str(nextFrontier)] = getHeuristic([n[0],n[1]],pgoal) + getDistance(E,[n[0],n[1]])

            fvals = frontier.values()
            if len(fvals) == 0:
                break
            min_val = min(fvals)
            nextFrontier = eval(frontier.keys()[fvals.index(min_val)])

            if nextFrontier[1] == pgoal:
                break

        path = getPath(E,pgoal)
        pathpnt = []
        pub_pathpnt = []
        for p in path:
            pub_pathpnt.append(IntList(p))
            pathpnt.append(p)

        path = Waypoints()
        path.data = pub_pathpnt
        # self.predator_path_pub.publish(predator_path)

        return pathpnt,path

    def findNeighbors(self,pnt):
        wpnts = np.copy(self.waypnt_vals).tolist()
        wpnts.remove(pnt)

        if len(self.obstacles) > 0:
            for p in self.obstacle_pnts:
                wpnts.remove([p[0],p[1]])
        neighbors = []
        close_pnt = wpnts[closest_node(pnt,wpnts)]
        dist = getHeuristic(pnt,close_pnt)
        d = 0
        while d < 1.5*dist:
            neighbors.append(close_pnt)
            wpnts.remove(close_pnt)
            close_pnt = wpnts[closest_node(pnt,wpnts)]
            d = getHeuristic(pnt,close_pnt)

        return neighbors

def getDistance(tree,pnt):
    xp,yp = pnt
    check = 0
    distance = 0
    while check == 0:
        for sublist in tree:
            if sublist[1] == [xp,yp]:
                xp,yp = sublist[0][0],sublist[0][1]
                if sublist[0] == [-1,-1]:
                    check = 1
                    break
                distance += getHeuristic([xp,yp],sublist[1])

    return distance

def getPath(tree,pnt):
    xp,yp = pnt
    check = 0
    path = []
    while check == 0:
        for sublist in tree:
            if sublist[1] == [xp,yp]:
                path.append([xp,yp])
                xp,yp = sublist[0][0],sublist[0][1]
                if sublist[0] == [-1,-1]:
                    check = 1
                    break
    path.reverse()
    return path

def getHeuristic(pcurrent,pgoal):
    xc,yc = pcurrent
    xg,yg = pgoal

    deltaX = xc - xg
    deltaY = yc - yg

    distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

    return distance

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

def points (p0, p1):
    x0, y0 = p0
    x1, y1 = p1

    dx = abs(x1-x0)
    dy = abs(y1-y0)
    if x0 < x1:
        sx = 1
    else:
        sx = -1


    if y0 < y1:
        sy = 1
    else:
        sy = -1
    err = dx-dy

    point_list = []
    while True:
        point_list.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break

        e2 = 2*err
        if e2 > -dy:
            # overshot in the y direction
            err = err - dy
            x0 = x0 + sx
        if e2 < dx:
            # overshot in the x direction
            err = err + dx
            y0 = y0 + sy

    return point_list

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

    getWaypoints = rospy.ServiceProxy("waypoints_fixed",waypoint)
    rospy.wait_for_service("waypoints_fixed")
    try:
        waypoint_list = getWaypoints()
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

    with open(os.path.join(rospack.get_path("maze_control"), "src", "obstacles.csv")) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            if len(row) == 0:
                break
            else:
                ic.obstacles.append(row[0])

    ic.prey_offset = prey_offset.offset.data
    ic.predator_offset = predator_offset.offset.data
    ic.waypntcb(waypoint_list)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
