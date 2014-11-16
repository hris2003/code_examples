'''pointcloud
Created on Jul 26, 2013

@author: meka
'''

import sys
import rospy

import cv
from cv import *
from sensor_msgs.msg import Image as Image1
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
from sensor_msgs.msg import PointCloud,PointCloud2
from geometry_msgs.msg import Point
import time
import numpy as np
#import easygui as eg

import math
import struct
from sensor_msgs.msg import PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)
def isnan(pt):
    return np.isnan(pt[0]) and np.isnan(pt[1]) and np.isnan(pt[2])
def read_points(cloud, field_names=None, skip_nans=False):
    assert(cloud)
    pts=[]
    fmt = _get_struct_fmt(cloud, field_names)
    width, height, point_step, row_step, data, unpack_from, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, struct.unpack_from, math.isnan
    if skip_nans:
        for v in xrange(height):
            offset = row_step * v
            for u in xrange(width):
                p = unpack_from(fmt, data, offset)
                has_nan = False
                for v in p:
                    if isnan(v):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
                offset += point_step
    else:
        for v in xrange(height):
            offset = row_step * v
            for u in xrange(width):
                yield unpack_from(fmt, data, offset)
                offset += point_step
def read_point(cloud, x=-1,y=-1,field_names=None):
    assert(cloud)
    fmt = _get_struct_fmt(cloud, field_names)
    width, height, point_step, row_step, data, unpack_from, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, struct.unpack_from, math.isnan
    offset = row_step * x + y*point_step
    return unpack_from(fmt, data, offset)
def _get_struct_fmt(cloud, field_names=None):
    fmt = '>' if cloud.is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


'''
Created on Jul 26, 2013

@author: meka
'''

import sys
import rospy
import cv
import cv2
import numpy as np
from cv import *
from sensor_msgs.msg import Image as Image1
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
import Tkinter

import time
#import easygui as eg

COLOR_RANGE={
'yellow': (Scalar(20, 0, 100, 0), Scalar(100, 255, 255, 0)),\
'red': (Scalar(170, 120, 120, 0), Scalar(180, 255, 255, 0)),\
'blue': (Scalar( 90 , 84 , 69 , 0 ), Scalar( 120 , 255 , 255 , 0)),\
'green': (Scalar( 40 , 80 , 32 , 0), Scalar( 70 , 255 , 255 , 0)),\
'orange': (Scalar( 160 , 100 , 47 , 0 ), Scalar( 179 , 255 , 255 , 0 ))\

}

DISPLAY_COLOR={
'yellow':CV_RGB(255,255,0)
,'red':CV_RGB(255,0,0)
,'blue':CV_RGB(0,0,255)
,'green':CV_RGB(0,110,0)

}
def putText2Img(img, text, x, y):
        text_color = (255,0,0) #color as (B,G,R)
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0, 3, 8) #Creates a font
        cv.PutText(img, text, (x,y), font, text_color)
        
class ColorSliderWindow(Tkinter.Tk):
    def __init__(self, parent, vec_color, ct, trackerthread):
        '''
        Constructor
        '''
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.colors = vec_color
        self.color_ranges = COLOR_RANGE;
        self.ct = ct
        self.trackthread = trackerthread
        self.initGUI()
        
    def getRanges(self, color):
        return self.color_ranges[color]
    
    def initGUI(self):
        self.grid()
        
        self.redminHvar = Tkinter.DoubleVar()
        self.redminHvar.set(self.color_ranges[self.colors[0]][0][0]);
        self.redmaxHvar = Tkinter.DoubleVar()
        self.redmaxHvar.set(self.color_ranges[self.colors[0]][1][0]);
        
        self.secondminHvar = Tkinter.DoubleVar()
        self.secondminHvar.set(self.color_ranges[self.colors[1]][0][0]);
        self.secondmaxHvar = Tkinter.DoubleVar()
        self.secondmaxHvar.set(self.color_ranges[self.colors[1]][1][0]);
        
        self.thirdminHvar = Tkinter.DoubleVar()
        self.thirdminHvar.set(self.color_ranges[self.colors[2]][0][0]);
        self.thirdmaxHvar = Tkinter.DoubleVar()
        self.thirdmaxHvar.set(self.color_ranges[self.colors[2]][1][0]);
        
        self.redminSvar = Tkinter.DoubleVar()
        self.redminSvar.set(self.color_ranges[self.colors[0]][0][1]);
        self.redmaxSvar = Tkinter.DoubleVar()
        self.redmaxSvar.set(self.color_ranges[self.colors[0]][1][1]);
        
        self.secondminSvar = Tkinter.DoubleVar()
        self.secondminSvar.set(self.color_ranges[self.colors[1]][0][1]);
        self.secondmaxSvar = Tkinter.DoubleVar()
        self.secondmaxSvar.set(self.color_ranges[self.colors[1]][1][1]);
        
        self.thirdminSvar = Tkinter.DoubleVar()
        self.thirdminSvar.set(self.color_ranges[self.colors[2]][0][1]);
        self.thirdmaxSvar = Tkinter.DoubleVar()
        self.thirdmaxSvar.set(self.color_ranges[self.colors[2]][1][1]);        
        
        self.redminVvar = Tkinter.DoubleVar()
        self.redminVvar.set(self.color_ranges[self.colors[0]][0][2]);
        self.redmaxVvar = Tkinter.DoubleVar()
        self.redmaxVvar.set(self.color_ranges[self.colors[0]][1][2]);
        
        self.secondminVvar = Tkinter.DoubleVar()
        self.secondminVvar.set(self.color_ranges[self.colors[1]][0][2]);
        self.secondmaxVvar = Tkinter.DoubleVar()
        self.secondmaxVvar.set(self.color_ranges[self.colors[1]][1][2]);
        
        self.thirdminVvar = Tkinter.DoubleVar()
        self.thirdminVvar.set(self.color_ranges[self.colors[2]][0][2]);
        self.thirdmaxVvar = Tkinter.DoubleVar()
        self.thirdmaxVvar.set(self.color_ranges[self.colors[2]][1][2]);        
        
        
        r = 0;
        #first color
        redLbl = Tkinter.Label(self, text = self.colors[0])
        redLbl.grid(column= 0, row = r+1)
        
        #r += 1;
        redminHLbl = Tkinter.Label(self, text="minH")
        redminHLbl.grid(column=1, row=r)
        redmaxHLbl = Tkinter.Label(self, text="maxH")
        redmaxHLbl.grid(column=2, row=r)
        redminSLbl = Tkinter.Label(self, text="minS")
        redminSLbl.grid(column=3, row=r)
        redmaxSLbl = Tkinter.Label(self, text="maxS")
        redmaxSLbl.grid(column=4, row=r)
        redminVLbl = Tkinter.Label(self, text="minV")
        redminVLbl.grid(column=5, row=r)
        redmaxVLbl = Tkinter.Label(self, text="maxV")
        redmaxVLbl.grid(column=6, row=r)

        
        r+=1;

        redminHScale = Tkinter.Scale(self, variable = self.redminHvar, from_=0, to=255)
        redminHScale.grid(column=1, row =r)
        redmaxHScale = Tkinter.Scale(self, variable = self.redmaxHvar, from_=0, to=255)
        redmaxHScale.grid(column=2, row =r)
        redminHScale.bind("<ButtonRelease-1>", self.updateRanges)
        redmaxHScale.bind("<ButtonRelease-1>", self.updateRanges)
        redminSScale = Tkinter.Scale(self, variable = self.redminSvar, from_=0, to=255)
        redminSScale.grid(column=3, row =r)
        redmaxSScale = Tkinter.Scale(self, variable = self.redmaxSvar, from_=0, to=255)
        redmaxSScale.grid(column=4, row =r)
        redminSScale.bind("<ButtonRelease-1>", self.updateRanges)
        redmaxSScale.bind("<ButtonRelease-1>", self.updateRanges)
        redminVScale = Tkinter.Scale(self, variable = self.redminVvar, from_=0, to=255)
        redminVScale.grid(column=5, row =r)
        redmaxVScale = Tkinter.Scale(self, variable = self.redmaxVvar, from_=0, to=255)
        redmaxVScale.grid(column=6, row =r)
        redminVScale.bind("<ButtonRelease-1>", self.updateRanges)
        redmaxVScale.bind("<ButtonRelease-1>", self.updateRanges)
        
        
        r += 1;
        #second color
        secondLbl = Tkinter.Label(self, text = self.colors[1])
        secondLbl.grid(column= 0, row = r+1)
        
        #r += 1;
        secondminHLbl = Tkinter.Label(self, text="minH")
        secondminHLbl.grid(column=1, row=r)
        secondmaxHLbl = Tkinter.Label(self, text="maxH")
        secondmaxHLbl.grid(column=2, row=r)
        secondminSLbl = Tkinter.Label(self, text="minS")
        secondminSLbl.grid(column=3, row=r)
        secondmaxSLbl = Tkinter.Label(self, text="maxS")
        secondmaxSLbl.grid(column=4, row=r)
        secondminVLbl = Tkinter.Label(self, text="minV")
        secondminVLbl.grid(column=5, row=r)
        secondmaxVLbl = Tkinter.Label(self, text="maxV")
        secondmaxVLbl.grid(column=6, row=r)
        
        
        r+=1;
        secondminHScale = Tkinter.Scale(self, variable = self.secondminHvar, from_=0, to=255)
        secondminHScale.grid(column=1, row =r)
        secondmaxHScale = Tkinter.Scale(self, variable = self.secondmaxHvar, from_=0, to=255)
        secondmaxHScale.grid(column=2, row =r)
        secondminHScale.bind("<ButtonRelease-1>", self.updateRanges)
        secondmaxHScale.bind("<ButtonRelease-1>", self.updateRanges)
        secondminSScale = Tkinter.Scale(self, variable = self.secondminSvar, from_=0, to=255)
        secondminSScale.grid(column=3, row =r)
        secondmaxSScale = Tkinter.Scale(self, variable = self.secondmaxSvar, from_=0, to=255)
        secondmaxSScale.grid(column=4, row =r)
        secondminSScale.bind("<ButtonRelease-1>", self.updateRanges)
        secondmaxSScale.bind("<ButtonRelease-1>", self.updateRanges)
        secondminVScale = Tkinter.Scale(self, variable = self.secondminVvar, from_=0, to=255)
        secondminVScale.grid(column=5, row =r)
        secondmaxVScale = Tkinter.Scale(self, variable = self.secondmaxVvar, from_=0, to=255)
        secondmaxVScale.grid(column=6, row =r)
        secondminVScale.bind("<ButtonRelease-1>", self.updateRanges)
        secondmaxVScale.bind("<ButtonRelease-1>", self.updateRanges)
        
        r += 1;
        #third color
        thirdLbl = Tkinter.Label(self, text = self.colors[2])
        thirdLbl.grid(column= 0, row = r+1)
        
        #r += 1;
        thirdminHLbl = Tkinter.Label(self, text="minH")
        thirdminHLbl.grid(column=1, row=r)
        thirdmaxHLbl = Tkinter.Label(self, text="maxH")
        thirdmaxHLbl.grid(column=2, row=r)
        thirdminSLbl = Tkinter.Label(self, text="minS")
        thirdminSLbl.grid(column=3, row=r)
        thirdmaxSLbl = Tkinter.Label(self, text="maxS")
        thirdmaxSLbl.grid(column=4, row=r)
        thirdminVLbl = Tkinter.Label(self, text="minV")
        thirdminVLbl.grid(column=5, row=r)
        thirdmaxVLbl = Tkinter.Label(self, text="maxV")
        thirdmaxVLbl.grid(column=6, row=r)
        
        
        r+=1;
        thirdminHScale = Tkinter.Scale(self, variable = self.thirdminHvar, from_=0, to=255)
        thirdminHScale.grid(column=1, row =r)
        thirdmaxHScale = Tkinter.Scale(self, variable = self.thirdmaxHvar, from_=0, to=255)
        thirdmaxHScale.grid(column=2, row =r)
        thirdminHScale.bind("<ButtonRelease-1>", self.updateRanges)
        thirdmaxHScale.bind("<ButtonRelease-1>", self.updateRanges)
        
        thirdminSScale = Tkinter.Scale(self, variable = self.thirdminSvar, from_=0, to=255)
        thirdminSScale.grid(column=3, row =r)
        thirdmaxSScale = Tkinter.Scale(self, variable = self.thirdmaxSvar, from_=0, to=255)
        thirdmaxSScale.grid(column=4, row =r)
        thirdminSScale.bind("<ButtonRelease-1>", self.updateRanges)
        thirdmaxSScale.bind("<ButtonRelease-1>", self.updateRanges)
        
        thirdminVScale = Tkinter.Scale(self, variable = self.thirdminVvar, from_=0, to=255)
        thirdminVScale.grid(column=5, row =r)
        thirdmaxVScale = Tkinter.Scale(self, variable = self.thirdmaxVvar, from_=0, to=255)
        thirdmaxVScale.grid(column=6, row =r)
        thirdminVScale.bind("<ButtonRelease-1>", self.updateRanges)
        thirdmaxVScale.bind("<ButtonRelease-1>", self.updateRanges)
        
        r+=1;
        updateBtn = Tkinter.Button(self, text="Quit")
        updateBtn.grid(column=0, row=r)
        updateBtn.bind("<ButtonRelease-1>", self.quit)
        
    def updateRanges(self, event):
        myrange = self.color_ranges[self.colors[0]][0]
        myrange2 = self.color_ranges[self.colors[0]][1]
        self.color_ranges[self.colors[0]]=(Scalar(self.redminHvar.get(), self.redminSvar.get(), self.redminVvar.get(), myrange[3]),Scalar(self.redmaxHvar.get(), self.redmaxSvar.get(), self.redmaxVvar.get(), myrange2[3]))
        
        myrange = self.color_ranges[self.colors[1]][0]
        myrange2 = self.color_ranges[self.colors[1]][1]
        self.color_ranges[self.colors[1]]=(Scalar(self.secondminHvar.get(), self.secondminSvar.get(), self.secondminVvar.get(), myrange[3]),Scalar(self.secondmaxHvar.get(), self.secondmaxSvar.get(),self.secondmaxVvar.get(), myrange2[3]))
        
        myrange = self.color_ranges[self.colors[2]][0]
        myrange2 = self.color_ranges[self.colors[2]][1]
        self.color_ranges[self.colors[2]]=(Scalar(self.thirdminHvar.get(), self.thirdminSvar.get(), self.thirdminVvar.get(), myrange[3]),Scalar(self.thirdmaxHvar.get(), self.thirdmaxSvar.get(), self.thirdmaxVvar.get(), myrange2[3]))
        print "just update color range"
        self.ct.updateRanges(self.color_ranges)
        print self.color_ranges
#         self.color_ranges[self.colors[0]][0]=Scalar(self.redminHvar.get(), myrange[1], myrange[2], myrange[3])
#         
#         self.color_ranges[self.colors[0]][1] = Scalar(self.redmaxHvar.get(), myrange2[1], myrange2[2], myrange2[3])
        
    def quit(self, event):
        self.trackthread.stop()
        self.destroy()
        
class ColorTracker():
    
    def __init__(self, vec_color):
        #self.gui = None
        #if withSlider:
        #self.gui = Slider;#ColorSliderWindow(None, vec_color)
        self.color_ranges = COLOR_RANGE;
        
        pass
    
    def updateRanges(self, ranges):
        self.color_ranges = ranges
        
    def GetThresholdedImage(self, img, color):
    #returns thresholded image of the blue bottle
        imgHSV = CreateImage(GetSize(img), 8, 3)
    #converts a BGR image to HSV
        CvtColor(img, imgHSV, CV_BGR2HSV)
        imgThreshed = CreateImage(GetSize(img), 8, 1)
    #InRangeS takes source, lowerbound color, upperbound color and destination
    #It converts the pixel values lying within the range to 255 and stores it in
    #the destination
        #color = "blue"
        color_range=self.color_ranges[color]
#         if self.gui is not None:
#             color_range = self.gui.getRanges(color);
        h_min=color_range[0]
        h_max=color_range[1]
        InRangeS(imgHSV, h_min, h_max, imgThreshed)
        return imgThreshed
    
    def checkColorAtPos(self, img, color, cX, cY, radius):
        
        sub = cv.GetSubRect(img, (cX-radius/2, cY-radius/2, radius/2, radius/2))
        thresh = self.GetThresholdedImage(sub, color)
        #print thresh
        rsX, rsY = self.getColorCentrePosition(thresh)
        return rsX > -1 and rsY > -1
        
    def getColorCentrePosition(self, imgThresh):
        #Smooth(imgBlueThresh,imgBlueThresh,CV_GAUSSIAN,9,9) #Smooth the thresh image to remove noise
        Erode(imgThresh,imgThresh,None,5)# eroding removes small noises
        mat = GetMat(imgThresh)
#Calculating the moments
        moments = Moments(mat, 0) 
        area = GetCentralMoment(moments, 0, 0)
        moment10 = GetSpatialMoment(moments, 1, 0)
        moment01 = GetSpatialMoment(moments, 0,1)
        posX = -1;
        posY = -1;
        if(area > 1000): 
#Calculating the coordinate postition of the centroid
            posX = int(moment10 / area)
            posY = int(moment01 / area)
        return posX, posY

    def poll(self,img):
        current = np.asarray(img[:,:])
        #current = cv2.cvtColor(np.asarray(img[:,:]), cv2.COLOR_GRAY2BGR)
        current_gs = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)
        
        circles = cv2.HoughCircles(current_gs,cv2.cv.CV_HOUGH_GRADIENT,1,40,param1=100,param2=30,minRadius=20,maxRadius=40)
        #print "circle: ", circles
        if circles is None:
            return [];
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv.Circle(img,(i[0],i[1]),i[2],(0,255,0),1)  # draw the outer circle
            cv.Circle(img,(i[0],i[1]),2,(0,0,255),3)     # draw the center of the circle
        return circles[0];

    
#     def getCircleCenter(self, im):
#         gray = cv.CreateImage(cv.GetSize(im), 8, 1)
#         edges = cv.CreateImage(cv.GetSize(im), 8, 1)
#     
#         cv.CvtColor(im, gray, cv.CV_BGR2GRAY)
#         cv.Canny(gray, edges, 20, 55, 3)
#     
#         storage = cv.CreateMat(im.width, 1, cv.CV_32FC3)
#         cirs = cv.HoughCircles(gray, storage, cv.CV_HOUGH_GRADIENT, 1, 5, 200, 100, 0 , 0)
#         #storage = cv.HoughCircles(edges, storage, cv.CV_HOUGH_GRADIENT, 2, 25, 100, 40, 20, 200)
#         print "tracking circles...", cirs
#         for i in xrange(storage.width - 1):
#             radius = storage[i, 2]
#             center = (storage[i, 0], storage[i, 1])
#     
#             print "circle: ", (radius, center)
#     
#         return 0;
    
    def getDetectRect(self,img, c):
       
        imgBlueThresh = self.GetThresholdedImage(img, c)
        #Smooth(imgBlueThresh,imgBlueThresh,CV_GAUSSIAN,9,9) #Smooth the thresh image to remove noise
        Erode(imgBlueThresh,imgBlueThresh,None,2)# eroding removes small noises
        nextX, nextY = self.getColorCentrePosition(imgBlueThresh)
        
        return [nextX, nextY, nextX + 1, nextY + 1]
        
#     def checkColorExist(self, img, c):
#         color_rect = self.getDetectRect(img, c)
#         return (color_rect[0] >= 0) and (color_rect[1] >= 0)
#         
#     def checkColorAtPos(self, img, pos, c):
#         color_rect = self.getDetectRect(img, c)
#         print (color_rect[0] - pos[0]), (color_rect[1] - pos[1])
#         return (abs(color_rect[0] - pos[0]) < 40) and (abs(color_rect[1] - pos[1]) < 40)
    
class MyThreeColorTracker(Thread):
    def __init__(self, vec_color, ct,verbose=True):
        Thread.__init__(self)
        self.track_color={vec_color[0]:[-1,-1], vec_color[1]:[-1,-1], vec_color[2]:[-1,-1]}
        self.blob_3d_pos={}
        self.bridge = CvBridge()
        self.curr_image = None
        self.detect = [-1, -1, 0, 0]
        self.detect_time = time.time()
        self.verbose=verbose
        self.keepContinue=True
        self.ct = self.ct = ct;
        
        #to save video
        self.tab_kin = []
        self.kin_video = cv2.VideoWriter("ha_experiment_{}.avi".format(time.asctime()), cv2.cv.CV_FOURCC('F', 'M', 'P', '4'), 12, (640, 480));


    def writeVideo(self):
        for img in self.tab_kin:
            self.kin_video.write(np.asarray(img[:,:]))
            
    def get_track_result(self):
        return self.track_color
    def start(self):
        if self.verbose:
            print 'Starting MyThreeColorTrackerThread...'
            
        #rospy.init_node('my_tracker', anonymous=True,disable_signals=True) #allow Ctrl-C to master process
        self.kinect_sub =rospy.Subscriber("/kinect_top/rgb/image_rect_color",Image1,self.kinectCallback,queue_size=1, buff_size=2**24)
        self.cloud_sub =rospy.Subscriber("/kinect_top/depth/points",PointCloud2,self.cloudCallback)
        Thread.start(self) 
        
    def cloudCallback(self,cloud):
        for color in self.track_color:
            #print 'Testing with color:',color
            [y,x] = self.track_color[color]
            if not (x is -1 or y is -1):
                pt = read_point(cloud,x,y)
                #print 'lets iterate for color:',color,'at :',x,y
                #print 'COLOR TRACKER Color:',color,'pos:',np.array(pt)
                if not isnan(np.array(pt)):
                    self.blob_3d_pos[color]=np.array(pt)
    def kinectCallback(self,data):
    
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        self.tab_kin.append(cv_image);
        self.eye="middle"
        self.curr_image = cv_image

        circles = self.ct.poll(cv_image)
        
        for color in self.track_color:
            has_color = False
            #[nextX, nextY, w, h] = self.ct.getDetectRect(sub, color);#return a rectangle
            for pos in circles:
                has_color = self.ct.checkColorAtPos(cv_image, color, pos[0], pos[1], pos[2]);
                if has_color:
                    break;
            if not has_color:
                continue;
            
            self.eye = "kinect_top"
            self.track_color[color]=[cv.Round(.8*self.track_color[color][0]+.2*pos[0]), cv.Round(.8*self.track_color[color][1]+.2*pos[1])]
            self.detect_time = time.time()
            cv.Circle(cv_image, (self.track_color[color][0], self.track_color[color][1]), 3, DISPLAY_COLOR[color],5)
            #self.publish_color(color, self.blob_3d_pos[color], 'kinect_top_depth_optical_frame')
        if self.verbose:
            putText2Img(cv_image, "Kinect", 20, 20)
            #print self.track_color['red']
            cv.ShowImage("Kinect Image window", cv_image)
            cv.WaitKey(3)
            
    def stop(self):
        #self.keepContinue=False
        self.writeVideo();
        self.kin_video.release()
        print "MyThreeColorTracker stopped"
        self.kinect_sub.unregister()
        cv.DestroyAllWindows()
        #rospy.signal_shutdown('Exiting')
    
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()        
            

if __name__ == '__main__':
    
    color_vec =("red", "yellow", "blue")
    singleColortracker = ColorTracker(color_vec)
    mfd = MyThreeColorTracker(color_vec, singleColortracker, True)
    
    mfd.start()
    gui = ColorSliderWindow(None, color_vec, singleColortracker, mfd)
    gui.mainloop()

