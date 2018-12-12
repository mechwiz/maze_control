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
        self.avgpnts9x = []
        self.avgpnts9y = []
        self.avgpnts10x = []
        self.avgpnts10y = []
        self.avgpnts11x = []
        self.avgpnts11y = []
        self.avgpnts12x = []
        self.avgpnts12y = []
        self.polypnts = []
        self.polypnts1x = []
        self.polypnts1y = []
        self.polypnts2x = []
        self.polypnts2y = []
        self.polypnts3x = []
        self.polypnts3y = []
        self.polypnts4x = []
        self.polypnts4y = []
        self.polypnts5x = []
        self.polypnts5y = []
        self.polypnts6x = []
        self.polypnts6y = []
        self.polypnts7x = []
        self.polypnts7y = []
        self.polypnts8x = []
        self.polypnts8y = []
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
            # lower_birch = np.array([lowh,lows,lowv])
            # upper_birch = np.array([upph,upps,uppv])
            # lower_red = np.array([1,0,0])
            # upper_red = np.array([30,255,255])
            # lower_red = np.array([1,34,81])
            # upper_red = np.array([52,255,202])
            # lower_red = np.array([10,0,0])
            # upper_red = np.array([35,255,200])
            # lower_red = np.array([1,34,50])
            # upper_red = np.array([52,255,255])
            lower_birch = np.array([1,0,75]) #[1,20,120]
            upper_birch = np.array([33,128,255])
            lower_red = np.array([50,85,100])
            upper_red = np.array([180,255,255])

            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # img_original = cv2.flip(img_original,1)
            hsv = cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
            maskdots = cv2.inRange(hsv,lower_red, upper_red)
            maskbirch = cv2.inRange(hsv,lower_birch, upper_birch)
            # res =cv2.bitwise_and(img_original,img_original,mask= maskbirch)

            contour_dots = cv2.findContours(maskdots.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            contour_maze = cv2.findContours(maskbirch.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if len(contour_dots) > 0 and len(contour_maze)>0:
                cnts = max(contour_maze, key = cv2.contourArea)
                # print cnts
                # print '***************'
                peri = .01*cv2.arcLength(cnts, True)
                hull = cv2.convexHull(cnts,clockwise=True)
                approx = cv2.approxPolyDP(hull,peri,True)
                if len(approx)>=8:
                    cv2.drawContours(img_original, [hull], -1, (0, 255, 0), 3)
                    # res = cv2.drawContours(res, [hull], -1, (0, 255, 0), 3)

                    self.polypnts1x.append(approx[0][0][0])
                    self.polypnts1y.append(approx[0][0][1])
                    self.polypnts2x.append(approx[1][0][0])
                    self.polypnts2y.append(approx[1][0][1])
                    self.polypnts3x.append(approx[2][0][0])
                    self.polypnts3y.append(approx[2][0][1])
                    self.polypnts4x.append(approx[3][0][0])
                    self.polypnts4y.append(approx[3][0][1])
                    self.polypnts5x.append(approx[4][0][0])
                    self.polypnts5y.append(approx[4][0][1])
                    self.polypnts6x.append(approx[5][0][0])
                    self.polypnts6y.append(approx[5][0][1])
                    self.polypnts7x.append(approx[6][0][0])
                    self.polypnts7y.append(approx[6][0][1])
                    self.polypnts8x.append(approx[7][0][0])
                    self.polypnts8y.append(approx[7][0][1])
                    # print len(self.polypnts1x)
                    if len(self.polypnts1x) == 100:
                        self.polypnts = []
                        self.polypnts.append([stats.mode(self.polypnts1x)[0][0],stats.mode(self.polypnts1y)[0][0]])
                        self.polypnts.append([stats.mode(self.polypnts2x)[0][0],stats.mode(self.polypnts2y)[0][0]])
                        self.polypnts.append([stats.mode(self.polypnts3x)[0][0],stats.mode(self.polypnts3y)[0][0]])
                        self.polypnts.append([stats.mode(self.polypnts4x)[0][0],stats.mode(self.polypnts4y)[0][0]])
                        self.polypnts.append([stats.mode(self.polypnts5x)[0][0],stats.mode(self.polypnts5y)[0][0]])
                        self.polypnts.append([stats.mode(self.polypnts6x)[0][0],stats.mode(self.polypnts6y)[0][0]])
                        self.polypnts.append([stats.mode(self.polypnts7x)[0][0],stats.mode(self.polypnts7y)[0][0]])
                        self.polypnts.append([stats.mode(self.polypnts8x)[0][0],stats.mode(self.polypnts8y)[0][0]])
                        # print self.polypnts

                        self.polypnts1x = []
                        self.polypnts1y = []
                        self.polypnts2x = []
                        self.polypnts2y = []
                        self.polypnts3x = []
                        self.polypnts3y = []
                        self.polypnts4x = []
                        self.polypnts4y = []
                        self.polypnts5x = []
                        self.polypnts5y = []
                        self.polypnts6x = []
                        self.polypnts6y = []
                        self.polypnts7x = []
                        self.polypnts7y = []
                        self.polypnts8x = []
                        self.polypnts8y = []
                        # self.polypnts.append([approx[i][0][0],approx[i][0][1]])
                # print len(contour)
                goodlist=[]
                for c in contour_dots:
                    ((x,y),radius) = cv2.minEnclosingCircle(c)
                    # M = cv2.moments(c)
                    if radius > 1 and cv2.pointPolygonTest(cnts,(int(x),int(y)),True)>0:
                      # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                      # print int(x),int(y)
                      cv2.circle(img_original,(int(x),int(y)),int(radius),(0,255,0),2)
                      goodlist.append([int(x),int(y)])

                if len(goodlist) == 12:
                    pnt_hull = cv2.convexHull(np.array(goodlist,dtype='float32'),clockwise=True)
                    # print 'goodlist',goodlist
                    # print 'hull',pnt_hull
                # print len(goodlist)
                    # print pnt_hull
                    if len(pnt_hull) >= 8:
                        self.avgpnts1x.append(pnt_hull[0][0][0])
                        self.avgpnts1y.append(pnt_hull[0][0][1])
                        self.avgpnts2x.append(pnt_hull[1][0][0])
                        self.avgpnts2y.append(pnt_hull[1][0][1])
                        self.avgpnts3x.append(pnt_hull[2][0][0])
                        self.avgpnts3y.append(pnt_hull[2][0][1])

                        gl=np.copy(goodlist).tolist()
                        gl.remove([pnt_hull[2][0][0],pnt_hull[2][0][1]])
                        pnt = gl[find_closest([pnt_hull[2][0][0],pnt_hull[2][0][1]],gl)]
                        self.avgpnts4x.append(pnt[0])
                        self.avgpnts4y.append(pnt[1])


                        gl=np.copy(goodlist).tolist()
                        gl.remove([pnt_hull[3][0][0],pnt_hull[3][0][1]])
                        pnt = gl[find_closest([pnt_hull[3][0][0],pnt_hull[3][0][1]],gl)]
                        self.avgpnts5x.append(pnt[0])
                        self.avgpnts5y.append(pnt[1])


                        self.avgpnts6x.append(pnt_hull[3][0][0])
                        self.avgpnts6y.append(pnt_hull[3][0][1])
                        self.avgpnts7x.append(pnt_hull[4][0][0])
                        self.avgpnts7y.append(pnt_hull[4][0][1])
                        self.avgpnts8x.append(pnt_hull[5][0][0])
                        self.avgpnts8y.append(pnt_hull[5][0][1])
                        self.avgpnts9x.append(pnt_hull[6][0][0])
                        self.avgpnts9y.append(pnt_hull[6][0][1])

                        gl=np.copy(goodlist).tolist()
                        gl.remove([pnt_hull[6][0][0],pnt_hull[6][0][1]])
                        pnt = gl[find_closest([pnt_hull[6][0][0],pnt_hull[6][0][1]],gl)]
                        self.avgpnts10x.append(pnt[0])
                        self.avgpnts10y.append(pnt[1])

                        gl=np.copy(goodlist).tolist()
                        gl.remove([pnt_hull[7][0][0],pnt_hull[7][0][1]])
                        pnt = gl[find_closest([pnt_hull[7][0][0],pnt_hull[7][0][1]],gl)]
                        self.avgpnts11x.append(pnt[0])
                        self.avgpnts11y.append(pnt[1])

                        self.avgpnts12x.append(pnt_hull[7][0][0])
                        self.avgpnts12y.append(pnt_hull[7][0][1])

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
                        self.avgpnt.append([stats.mode(self.avgpnts9x)[0][0],stats.mode(self.avgpnts9y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts10x)[0][0],stats.mode(self.avgpnts10y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts11x)[0][0],stats.mode(self.avgpnts11y)[0][0]])
                        self.avgpnt.append([stats.mode(self.avgpnts12x)[0][0],stats.mode(self.avgpnts12y)[0][0]])

                        # print self.avgpnt
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
                        self.avgpnts9x = []
                        self.avgpnts9y = []
                        self.avgpnts10x = []
                        self.avgpnts10y = []
                        self.avgpnts11x = []
                        self.avgpnts11y = []
                        self.avgpnts12x = []
                        self.avgpnts12y = []

            # if len(contour_maze) > 0:

            #     cnts = max(contour, key = cv2.contourArea)
            #     peri = .01*cv2.arcLength(cnts, True)
            #     hull = cv2.convexHull(cnts,clockwise=True)
            #     approx = cv2.approxPolyDP(hull,peri,True)

            #     if len(approx) >= 8:

            #         self.avgpnts1x.append(approx[0][0][0])
            #         self.avgpnts1y.append(approx[0][0][1])
            #         self.avgpnts2x.append(approx[1][0][0])
            #         self.avgpnts2y.append(approx[1][0][1])
            #         self.avgpnts3x.append(approx[2][0][0])
            #         self.avgpnts3y.append(approx[2][0][1])
            #         self.avgpnts4x.append(approx[3][0][0])
            #         self.avgpnts4y.append(approx[3][0][1])
            #         self.avgpnts5x.append(approx[4][0][0])
            #         self.avgpnts5y.append(approx[4][0][1])
            #         self.avgpnts6x.append(approx[5][0][0])
            #         self.avgpnts6y.append(approx[5][0][1])
            #         self.avgpnts7x.append(approx[6][0][0])
            #         self.avgpnts7y.append(approx[6][0][1])
            #         self.avgpnts8x.append(approx[7][0][0])
            #         self.avgpnts8y.append(approx[7][0][1])

            #         cv2.drawContours(img_original, [hull], -1, (0, 255, 0), 3)
            #         # res = cv2.drawContours(res, [hull], -1, (0, 255, 0), 3)

            #         if len(self.avgpnts1x) == 100:
            #             self.avgpnt = []
            #             self.avgpnt.append([stats.mode(self.avgpnts1x)[0][0],stats.mode(self.avgpnts1y)[0][0]])
            #             self.avgpnt.append([stats.mode(self.avgpnts2x)[0][0],stats.mode(self.avgpnts2y)[0][0]])
            #             self.avgpnt.append([stats.mode(self.avgpnts3x)[0][0],stats.mode(self.avgpnts3y)[0][0]])
            #             self.avgpnt.append([stats.mode(self.avgpnts4x)[0][0],stats.mode(self.avgpnts4y)[0][0]])
            #             self.avgpnt.append([stats.mode(self.avgpnts5x)[0][0],stats.mode(self.avgpnts5y)[0][0]])
            #             self.avgpnt.append([stats.mode(self.avgpnts6x)[0][0],stats.mode(self.avgpnts6y)[0][0]])
            #             self.avgpnt.append([stats.mode(self.avgpnts7x)[0][0],stats.mode(self.avgpnts7y)[0][0]])
            #             self.avgpnt.append([stats.mode(self.avgpnts8x)[0][0],stats.mode(self.avgpnts8y)[0][0]])

            #             print self.avgpnt
            #             self.avgpnts1x = []
            #             self.avgpnts1y = []
            #             self.avgpnts2x = []
            #             self.avgpnts2y = []
            #             self.avgpnts3x = []
            #             self.avgpnts3y = []
            #             self.avgpnts4x = []
            #             self.avgpnts4y = []
            #             self.avgpnts5x = []
            #             self.avgpnts5y = []
            #             self.avgpnts6x = []
            #             self.avgpnts6y = []
            #             self.avgpnts7x = []
            #             self.avgpnts7y = []
            #             self.avgpnts8x = []
            #             self.avgpnts8y = []

            if len(self.avgpnt)>1 and len(self.polypnts)>1:
                # for pnt in self.avgpnt:
                #         cv2.circle(img_original,(pnt[0],pnt[1]),5,(255,0,0),-1)
                for pnt in self.polypnts:
                    cv2.circle(img_original,(pnt[0],pnt[1]),5,(255,0,0),-1)

                allpts = []

                tl = points(self.avgpnt[0],self.avgpnt[11])
                skp = len(tl)/8.0
                cnt = 0
                tla = []
                while cnt < len(tl)-1:
                    tla.append(tl[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                tla.append(tl[-1])

                bl = points(self.avgpnt[1],self.avgpnt[2])
                skp = len(bl)/8.0
                cnt = 0
                bla = []
                while cnt < len(bl)-1:
                    bla.append(bl[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                bla.append(bl[-1])

                lvl = 8.0
                for i in range(9):
                    left = points(bla[i],tla[i])
                    skp = len(left)/lvl
                    cnt = 0
                    while cnt < len(left)-1:
                        wp = left[int(np.round(cnt))]
                        allpts.append(IntList(wp))
                        cv2.circle(img_original,(int(wp[0]),int(wp[1])),5,(0,0,255),-1)
                        cnt+=skp
                    wp = left[-1]
                    allpts.append(IntList(wp))
                    cv2.circle(img_original,(int(wp[0]),int(wp[1])),5,(0,0,255),-1)
                    lvl+=1

                tml = points(self.avgpnt[10],self.avgpnt[9])
                skp = len(tml)/3.0
                cnt = 0
                tmla = []
                while cnt < len(tml)-1:
                    tmla.append(tml[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                tmla.append(tml[-1])

                bml = points(self.avgpnt[3],self.avgpnt[4])
                skp = len(bml)/3.0
                cnt = 0
                bmla = []
                while cnt < len(bml)-1:
                    bmla.append(bml[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                bmla.append(bml[-1])

                tmh = points(self.avgpnt[11],self.avgpnt[8])
                skp = len(tmh)/4.0
                cnt = 0
                tmha = []
                while cnt < len(tmh)-1:
                    tmha.append(tmh[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                tmha.append(tmh[-1])

                bmh = points(self.avgpnt[2],self.avgpnt[5])
                skp = len(bmh)/4.0
                cnt = 0
                bmha = []
                while cnt < len(bmh)-1:
                    bmha.append(bmh[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                bmha.append(bmh[-1])

                lvl = 15.0
                low = True
                i = 0
                j = 1
                for k in range(7):
                    if low == True:
                        center = points(bmla[i],tmla[i])
                        i+=1
                    else:
                        center = points(bmha[j],tmha[j])
                        j+=1
                    low = not low
                    skp = len(center)/lvl
                    cnt = 0
                    while cnt < len(center)-1:
                        wp = center[int(np.round(cnt))]
                        allpts.append(IntList(wp))
                        cv2.circle(img_original,(int(wp[0]),int(wp[1])),5,(0,0,255),-1)
                        cnt+=skp
                    wp = center[-1]
                    allpts.append(IntList(wp))
                    cv2.circle(img_original,(int(wp[0]),int(wp[1])),5,(0,0,255),-1)
                    if lvl == 15.0:
                        lvl+=1
                    else:
                        lvl-=1

                tr = points(self.avgpnt[8],self.avgpnt[7])
                skp = len(tr)/8.0
                cnt = 0
                tra = []
                while cnt < len(tr)-1:
                    tra.append(tr[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                tra.append(tr[-1])

                br = points(self.avgpnt[5],self.avgpnt[6])
                skp = len(br)/8.0
                cnt = 0
                bra = []
                while cnt < len(tr)-1:
                    bra.append(br[int(np.round(cnt))])
                    # cv2.circle(img_original,(wp[0],wp[1]),5,(0,0,255),2)
                    cnt+=skp
                bra.append(br[-1])

                lvl = 16.0
                for i in range(9):
                    right = points(bra[i],tra[i])
                    skp = len(right)/lvl
                    cnt = 0
                    while cnt < len(right)-1:
                        wp = right[int(np.round(cnt))]
                        allpts.append(IntList(wp))
                        cv2.circle(img_original,(int(wp[0]),int(wp[1])),5,(0,0,255),-1)
                        cnt+=skp
                    wp = right[-1]
                    allpts.append(IntList(wp))
                    cv2.circle(img_original,(int(wp[0]),int(wp[1])),5,(0,0,255),-1)
                    lvl-=1



                self.allpts = allpts
                for i in range(8):
                    self.allpts.append(IntList(self.polypnts[i]))
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

def find_closest(node,nodes):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

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
