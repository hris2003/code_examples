'''
Created on Oct 2, 2013

@author: meka
'''
import cv2, cv
from cv import *

#from cv import *
import math
import numpy as np
import time

COLOR_RANGE={
'yellow': (Scalar(25, 100, 100, 0), Scalar(30, 255, 255, 0)),\
'red': (Scalar(169, 120, 120, 0), Scalar(179, 255, 255, 0)),\
'blue': (Scalar( 80 , 120 , 69 , 0 ), Scalar( 129 , 255 , 255 , 0)),\
'green': (Scalar( 38 , 80 , 100 , 0), Scalar( 75 , 255 , 255 , 0)),\
'orange': (Scalar( 5 , 100 , 47 , 0 ), Scalar( 20 , 255 , 255 , 0 )),\
'violet': (Scalar( 130 , 100 , 120 , 0 ), Scalar( 167 , 255 , 255 , 0 ))\
}

DISPLAY_COLOR={
'yellow':CV_RGB(255,255,0)
,'red':CV_RGB(255,0,0)
,'blue':CV_RGB(0,0,255)
,'green':CV_RGB(0,110,0)
,'orange':CV_RGB(255,125,0)
, 'violet': CV_RGB(143,0,255)
}

class MyLineDetector(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self.color_ranges = COLOR_RANGE;
        
        
        pass

    def getValueInImage(self, image):
        '''
        1: red
        2: yellow
        3: blue
        4: green
        5: orange (not sure)
        '''
	rs_colors = []
        for c in self.color_ranges:
            x, y, w, h = self.getColorCentrePosition(self.getThresholdedImage(np.asarray(image[:,:]), c));
            if x > -1 and y > -1:
                #print "found color {} in the image".format(c)
                cv2.rectangle(np.asarray(image[:,:]),(int(x), int(y)), (int(x+5), int(y+5)),DISPLAY_COLOR[c],2)
		rs_colors.append(c)

                
        return rs_colors; 

    def getThresholdedImage(self, img, color):
        #returns thresholded image of the blue bottle
        #imgHSV = np.zeros(img.shape, np.uint8)
        #converts a BGR image to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV);
        #InRangeS takes source, lowerbound color, upperbound color and destination
        #It converts the pixel values lying within the range to 255 and stores it in
        #the destination
        color_range=self.color_ranges[color]
        h_min=color_range[0]
        h_max=color_range[1]
        imgThreshed = cv2.inRange(imgHSV, h_min, h_max)
        #cv2.imshow("red comp:", imgThreshed)
        return imgThreshed   
    
    def getColorCentrePosition(self, imgThresh):
        #Smooth(imgBlueThresh,imgBlueThresh,CV_GAUSSIAN,9,9) #Smooth the thresh image to remove noise
        #Erode(imgThresh,imgThresh,None,5)# eroding removes small noises
         #to get list of contours
        #size = np.shape(imgThresh)
        erode = cv2.erode(imgThresh,None,iterations = 1)
        dilate = cv2.dilate(erode,None,iterations = 1)
        
        moments = cv2.moments(dilate[5:-5,5:-5]) 
        area = moments['m00']
        moment10 = moments['m10']
        moment01 = moments['m01']
        posX = -1;
        posY = -1;
        if(area > 10000): 
		#Calculating the coordinate postition of the centroid
            posX = int(moment10 / area)
            posY = int(moment01 / area)
        return posX, posY, 0, 0;

    def detectRedContour(self, image):
        #img = cv2.imread(imagefilename)
        
        #cv2.imshow("original:", img)
        img = np.asarray(image[:,:])
	size = np.shape(img)
        thres = self.getThresholdedImage(img, 'red');
        
        
        #to get list of contours
        
        erode = cv2.erode(thres,None,iterations = 0)
        dilate = cv2.dilate(erode,None,iterations = 5)
        contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.findContours( thres, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        #print 'nb cnts: ', len(contours)
        redDir = 0;
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if (w < size[0]/2) and (h < size[1]/2):
                continue;
            #print "found rect: ", (x, y, w, h)
            redDir = self.checkRectDirection(w, h, size[0], size[1]);
#             box = cv2.cv.BoxPoints(rect)
#             print "found box: ", box
#             box = np.int0(box)
            
            cv2.rectangle(img,(int(x), int(y)), (int(x+w), int(y+h)),DISPLAY_COLOR['yellow'],2)
        
#        cv2.imshow("detect", img)
        
#         if redDir == 1:
#             player.say("horizontal", True);
#         if redDir == -1:
#             player.say("vertical", True);
#         if redDir == 0:
#             player.say("no direction", True);
#        cv2.waitKey()
#        
        return redDir;
    
    
    def checkRectDirection(self, rw, rh, iw, ih):
        if rw > rh: return 1;#horizontal
        if rw < rh: return -1;#vertical
        
        return 0;#no specific direction
        
    def detectLine(self, img):
        current = np.asarray(img[:,:])
#         thres = self.getThresholdedImage(img, 'red');
        
        
        #to get list of contours
        
#         erode = cv2.erode(thres,None,iterations = 0)
#         dilate = cv2.dilate(erode,None,iterations = 10)
#         contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
#         #cv2.findContours( thres, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
#         print 'nb cnts: ', len(contours)
        
        
        gray = cv2.cvtColor(current,cv2.COLOR_BGR2GRAY)
        #cv2.cvtColor(img, gray, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 120)
        #cv2.imshow("edges:", edges)
#         lines2 = cv2.HoughLines(edges, 1, math.pi/180, 10, 30, 10);
        lines = cv2.HoughLinesP(edges, 1, math.pi/180, 80, 30, 10);
        if lines is None:
#             print "no line found";
#             cv2.imshow("test:", img)
#             cv2.waitKey()
            return False;
        
#         print len(lines[0]), "lines found";
        
        #print lines2[0]
        for line in lines[0]:
            pt1 =(line[0],line[1]) 
            pt2 = (line[2],line[3])
            #angle = int(math.atan((line[1]-line[3])/(line[2]-line[0]))*180/math.pi)
            
            #cv2.line(img, pt1, pt2, DISPLAY_COLOR['yellow'], 3)
#             i = 0;
#             if contours is None:
#                 continue;
#             for cnt in contours:
#                 if cv2.pointPolygonTest(cnt, pt1, False) > -1:
#                     print line, " in cnt:", i;
#                     cv2.line(img, pt1, pt2, DISPLAY_COLOR['yellow'], 3)
#                     break;
#                 i+=1
                        
        if len(lines[0]) > 2:
            #print "line found"
            #cv2.line(img, (1,1), (50,10), DISPLAY_COLOR['red'], 5)
            return True;
        
        #print "no track found"
        return False;
    def detectLineFromFile(self, imagefilename):
        img = cv2.imread(imagefilename)
        self.detectLine(img);
        
        
        
        
