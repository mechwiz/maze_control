#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from scipy import stats
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from maze_control.msg import IntList, Waypoints

class image_overlay:

    def __init__(self):
        self.allpts=[]
        self.avgpnt = []
        self.avgpnts1x = []
        self.avgpnts1y = []
        self.avgpnts2x = []
        self.avgpnts2y = []
        self.avgpnts3x = []
        self.avgpnts3y = []
        self.avgpnts4x = []
        self.avgpnts4y = []
        self.avgpnts5x = []
        self.avgpnts5y = []
        self.avgpnts6x = []
        self.avgpnts6y = []
        self.avgpnts7x = []
        self.avgpnts7y = []
        self.avgpnts8x = []
        self.avgpnts8y = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imagecb)
        self.list_pub = rospy.Publisher("waypoints",Waypoints,queue_size=10)

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

            # lowh = cv2.getTrackbarPos('Lower Hue','Converted Image')
            # lows = cv2.getTrackbarPos('Lower Sat','Converted Image')
            # lowv = cv2.getTrackbarPos('Lower Value','Converted Image')
            # upph = cv2.getTrackbarPos('Upper Hue','Converted Image')
            # upps = cv2.getTrackbarPos('Upper Sat','Converted Image')
            # uppv= cv2.getTrackbarPos('Upper Value','Converted Image')
            # switch = '0 : OFF \n1 : ON'
            # cv2.createTrackbar(switch, 'Converted Image',0,1,nothing)
            # lower_red = np.array([lowh,lows,lowv])
            # upper_red = np.array([upph,upps,uppv])
            # lower_red = np.array([1,0,0])
            # upper_red = np.array([30,255,255])
            # lower_red = np.array([1,34,81])
            # upper_red = np.array([52,255,202])
            # lower_red = np.array([10,0,0])
            # upper_red = np.array([35,255,200])
            # lower_red = np.array([1,34,50])
            # upper_red = np.array([52,255,255])
            lower_red = np.array([1,32,85])
            upper_red = np.array([33,128,255])
            # lower_red = np.array([50,85,100])
            # upper_red = np.array([180,255,255])

            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # img_original = cv2.flip(img_original,1)
            hsv = cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,lower_red, upper_red)
            # res =cv2.bitwise_and(img_original,img_original,mask= mask)

            contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if len(contour) > 0:

                cnts = max(contour, key = cv2.contourArea)
                peri = .01*cv2.arcLength(cnts, True)
                hull = cv2.convexHull(cnts,clockwise=True)
                approx = cv2.approxPolyDP(hull,peri,True)

                if len(approx) >= 8:

                    self.avgpnts1x.append(approx[0][0][0])
                    self.avgpnts1y.append(approx[0][0][1])
                    self.avgpnts2x.append(approx[1][0][0])
                    self.avgpnts2y.append(approx[1][0][1])
                    self.avgpnts3x.append(approx[2][0][0])
                    self.avgpnts3y.append(approx[2][0][1])
                    self.avgpnts4x.append(approx[3][0][0])
                    self.avgpnts4y.append(approx[3][0][1])
                    self.avgpnts5x.append(approx[4][0][0])
                    self.avgpnts5y.append(approx[4][0][1])
                    self.avgpnts6x.append(approx[5][0][0])
                    self.avgpnts6y.append(approx[5][0][1])
                    self.avgpnts7x.append(approx[6][0][0])
                    self.avgpnts7y.append(approx[6][0][1])
                    self.avgpnts8x.append(approx[7][0][0])
                    self.avgpnts8y.append(approx[7][0][1])

                    cv2.drawContours(img_original, [hull], -1, (0, 255, 0), 3)
                    # res = cv2.drawContours(res, [hull], -1, (0, 255, 0), 3)

                    if len(self.avgpnts1x) == 100:
                        self.avgpnt = []
                        self.avgpnt.append([stats.mode(self.avgpnts1x)[0][0],stats.mode(self.avgpnts1y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts2x)[0][0],stats.mode(self.avgpnts2y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts3x)[0][0],stats.mode(self.avgpnts3y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts4x)[0][0],stats.mode(self.avgpnts4y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts5x)[0][0],stats.mode(self.avgpnts5y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts6x)[0][0],stats.mode(self.avgpnts6y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts7x)[0][0],stats.mode(self.avgpnts7y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts8x)[0][0],stats.mode(self.avgpnts8y)[0][0]])

                        print self.avgpnt
                        self.avgpnts1x = []
                        self.avgpnts1y = []
                        self.avgpnts2x = []
                        self.avgpnts2y = []
                        self.avgpnts3x = []
                        self.avgpnts3y = []
                        self.avgpnts4x = []
                        self.avgpnts4y = []
                        self.avgpnts5x = []
                        self.avgpnts5y = []
                        self.avgpnts6x = []
                        self.avgpnts6y = []
                        self.avgpnts7x = []
                        self.avgpnts7y = []
                        self.avgpnts8x = []
                        self.avgpnts8y = []

            if len(self.avgpnt)>1:
                for pnt in self.avgpnt:
                        cv2.circle(img_original,(pnt[0],pnt[1]),5,(255,0,0),-1)

                allpts = []

                tl = points(self.avgpnt[0],self.avgpnt[7])
                skp = len(tl)/9.0
                cnt = skp
                tla = []
                while cnt < len(tl)-1:
                    tla.append(tl[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp

                bl = points(self.avgpnt[1],self.avgpnt[2])
                skp = len(bl)/9.0
                cnt = skp
                bla = []
                while cnt < len(bl)-1:
                    bla.append(bl[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp

                lvl = 10.0
                for i in range(8):
                    left = points(bla[i],tla[i])
                    skp = len(left)/lvl
                    cnt = skp
                    while cnt < len(left)-1:
                        wp = left[int(np.round(cnt))]
                        allpts.append(IntList(wp))
                        cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),-1)
                        cnt+=skp
                    lvl+=1

                tm = points(self.avgpnt[7],self.avgpnt[6])
                skp = len(tm)/8.0
                cnt = skp
                tma = []
                while cnt < len(tm)-1:
                    tma.append(tm[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                tma.insert(0,tm[0])
                tma.append(tm[-1])

                bm = points(self.avgpnt[2],self.avgpnt[3])
                skp = len(bm)/8.0
                cnt = skp
                bma = []
                while cnt < len(bm)-1:
                    bma.append(bm[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                bma.insert(0,bm[0])
                bma.append(bm[-1])

                lvl = 18.0
                for i in range(9):
                    center = points(bma[i],tma[i])
                    skp = len(center)/lvl
                    cnt = skp
                    while cnt < len(center)-1:
                        wp = center[int(np.round(cnt))]
                        allpts.append(IntList(wp))
                        cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),-1)
                        cnt+=skp
                    if lvl == 18.0:
                        lvl-=1
                    else:
                        lvl+=1

                tr = points(self.avgpnt[6],self.avgpnt[5])
                skp = len(tr)/9.0
                cnt = skp
                tra = []
                while cnt < len(tr)-1:
                    tra.append(tr[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp

                br = points(self.avgpnt[3],self.avgpnt[4])
                skp = len(br)/9.0
                cnt = skp
                bra = []
                while cnt < len(tr)-1:
                    bra.append(br[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp

                lvl = 17.0
                for i in range(8):
                    right = points(bra[i],tra[i])
                    skp = len(right)/lvl
                    cnt = skp
                    while cnt < len(right)-1:
                        wp = right[int(np.round(cnt))]
                        allpts.append(IntList(wp))
                        cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),-1)
                        cnt+=skp
                    lvl-=1



                self.allpts = allpts
                for i in range(8):
                    self.allpts.append(IntList(self.avgpnt[i]))
                # print allpts


            cv2.imshow("Converted Image",img_original)
            # cv2.imshow("Converted Image",np.hstack([img_original,res]))

            k = cv2.waitKey(3)

            if k == ord('c') and len(self.avgpnt)>1:
                # print self.allpts
                alist = Waypoints()
                alist.data = self.allpts
                self.list_pub.publish(alist)
                rospy.loginfo('Points Captured')
                # print s1.data

        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

def nothing(x):
    pass

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
    rospy.init_node('image_overlay', anonymous=False)
    ic = image_overlay()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
