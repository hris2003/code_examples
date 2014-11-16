#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from cv_bridge import CvBridge, CvBridgeError
import cv2, cv
import os, os.path, sys, time;
from LineDetector import MyLineDetector
from SoundPlayer import SoundPlayClient
import numpy as np
import random

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui



# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected

DISPLAY_COLOR={
'yellow':(0,255,255)
,'red':(0,0,255)
,'blue':(255,0,0)
,'green':(0,255,0)
,'orange':(0,125,255)
, 'violet': (255,0,143)
, 'white': (0,0,0)
}
COLOR_VALUE={
'yellow':2
,'red':1
,'blue':3
,'green':4
,'orange':5
, 'violet': 0
, 'white': -1
}

COMMENTS = ["Relax a bit.","Do not stress yourself.","Try to stay on track" ]
class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self, folder):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()
		
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)

		self.bridge = CvBridge()
		sFolder = str(folder) + "/img"
        	if os.path.exists(sFolder) == False:
            		os.mkdir(sFolder) ;
		self.folder = sFolder;
		filename = str(folder) + "/Record_DisplayController_{0}.txt".format(time.asctime())
		self.f = open(filename, "w")
		print "curr folder: ", os.path.abspath(self.folder)
		self.it = 0;
		self.curDirection = False;#unclear direction
		self.newDirection = False;
		
		#print "init: ", time.time(), self.dir_last;
		self.dir_vec = [0]
		self.myLineDetector = MyLineDetector();
		self.player = SoundPlayClient()
		#self.player.start();
		self.cv_image = None;
		self.video = None;
		self.dt_colors = []
		self.level = 0;#0:easy, 1: difficult
		self.colorSteps = [["violet", "red", "violet", "yellow", "violet", "blue", "violet"],["violet", "red", "yellow", "blue", "green", "violet"]];
		self.step = -1;
		self.isVocal = True
		self.count_offtrack = 0;
		self.count_offMarker = 0;
		self.targetColorImg = np.zeros((300,300,3), np.uint8);
		self.targetColorImg[:,:] = DISPLAY_COLOR["white"];
		self.drawNumber(self.targetColorImg, -1);

		cv2.imshow("Target Color", self.targetColorImg);
		self.isEasyPerson = True;
		self.currentCount = 0;
		self.trackCount = 0;
		self.dir_last = 0;
		self.duration = 0;
		self.max_Duration = 5;
		self.isDetecting = False;
		self.msgResult = "";
		self.curComment = 0;

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def toggleVocal(self):
		self.isVocal = not self.isVocal;
		self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "isVocal", self.isVocal)));
		print "Current vocal: {}\tCurrent level: {}".format( self.isVocal, self.level);

	def toggleLevel(self):

		self.level = 1 - self.level;
		self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "isLevel", self.level)));
		print "Current vocal: {}\tCurrent level: {}".format( self.isVocal, self.level);

	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def calculateDirection(self, newDirection):
		#print "detected direction: ", newDirection;
		self.curDirection = self.curDirection or newDirection;
	#to close the record file
	def closeRecordFile(self):
		self.f.close();

	def enableDetecting(self):
		self.isDetecting = True;
		self.f.write(('{0}\t{1}\n'.format(time.asctime(), "isDetecting")));
	def disableDetecting(self):
		self.f.write(('{0}\t{1}\n'.format(time.asctime(), "is NOT Detecting")));
		self.isDetecting = False;
	def toggleDetecting(self):
		self.isDetecting = not self.isDetecting;
		if self.isDetecting:
			self.f.write('\n{0}\t{1}\n'.format(time.asctime(), "Begin counting time."));
			self.resetStep();
			self.player.say("Begin counting time.",True)
			self.f.write(('{0}\t{1}\n'.format(time.asctime(), "Current vocal: {}\tCurrent level: {}".format( self.isVocal, self.level))));
			height , width , layers =  np.asarray(self.cv_image[:,:]).shape
			name = str(self.folder) + "/Record_Video_{0}.avi".format(time.asctime())
			print "video name: ", name;
			self.f.write('{0}\t{1}\n'.format(time.asctime(), "Video name: {}".format(name)));
			self.video = cv2.VideoWriter(name,cv2.cv.CV_FOURCC('F', 'M', 'P', '4'),20,(width,height))
		else:	
			self.video.release();
			self.msgResult = self.reportResult();
			self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(),  "Vocal message:", self.msgResult)));
			self.f.write(('{0}\t{1}\t{2}\t{3}\n'.format(time.asctime(),  "Performance", self.count_offtrack, self.count_offMarker)));
			self.player.say(self.msgResult, True);

		self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "Detecting is", self.isDetecting)));
		

	def reportResult(self):
		msg = "";
		if self.step > -1:
			msg = "After {} minutes, you made {} step out of {}, with about {} minutes off track.".format(self.duration, self.step, len(self.colorSteps[self.level]),\
			int((self.count_offtrack+30)/60));
		else: msg = "After {} minutes, you finished the whole drive course with about {} minutes off track. "\
		.format(self.duration, int((self.count_offtrack+30)/60));

		if self.duration<3 and self.step == -1:
			msg += "Congratulations. You are among the best drivers."
			return msg;

		if self.isEasyPerson and self.step == -1:
			msg += "You did a very good job. And you will make it better the next time."
			return msg;

		if self.count_offtrack * 2 > (self.duration * 60):
			msg += "But more than half of the time, you went off track."
		if self.count_offMarker * 2 > (len(self.colorSteps[self.level])):
			msg += "And your drone moved too much out of the markers."
		if (self.count_offtrack * 2 <= (self.duration * 60)) and (self.count_offMarker * 2 <= (len(self.colorSteps[self.level]))):
			msg += "You did a good job. But you should try harder the next time."

		return msg;

	def drawNumber(self, img, val):
		cv2.putText(img, "{}".format(val), (20, 280),\
                cv2.FONT_HERSHEY_PLAIN, 25.0, (0,0,0),\
                thickness=15, lineType=cv2.CV_AA);

	#use by rospy to update the duration
	def updateDuration(self, event):
		if not self.isDetecting:
			return;

		if self.duration == self.max_Duration:
			self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "Vocal message:","{} minutes passed! You should stop now.".format(self.duration))));
			self.player.say("{} minutes passed! You should stop now.".format(self.duration),True);
			return;

		self.duration += 1;
		self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "Vocal message:","{} minutes passed!".format(self.duration))));
		self.player.say("{} minutes passed!".format(self.duration),True);
			
		print "{} minutes passed!".format(self.duration);

		if self.duration == self.max_Duration:
			self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "Vocal message:","{} minutes passed! You should stop now.".format(self.duration))));
			self.player.say("{} minutes passed! You should stop now.".format(self.duration),True);
			
		return;

	# Use by rospy.Timer to call back periodically in order to update the direction
	def updateDir(self, event):
		

		if self.step == -1:
			return;
		if not self.isDetecting:
			return;
		if self.duration == self.max_Duration:
			#self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "Vocal message:","{} minutes passed! You should stop now.".format(self.duration))));
			#self.player.say("{} minutes passed! You should stop now.".format(self.duration),True);
			return;
		if self.curDirection == False:
			self.count_offtrack += 1;
			self.trackCount += 1;
		else:
			self.trackCount = 0;

            	if (self.trackCount % 10 == 3):
			if (self.curComment == len(COMMENTS)):
				self.curComment = 0;
				random.shuffle(COMMENTS);
			
			myMsg = COMMENTS[self.curComment];
			self.curComment += 1;
			self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "Vocal message:",myMsg)));
                	if self.isVocal: self.player.say(myMsg, True) 
		
		print "updated direction ", self.curDirection
		self.dir_vec.append(self.curDirection)
		self.curDirection = False;
		return True;

	def resetStep(self):
		self.step = 0;#to begin the loop
		self.duration = 0;
		self.count_offtrack = 0;
		self.count_offMarker = 0;
		self.msgResult = "";
		self.targetColorImg[:,:] = DISPLAY_COLOR[self.colorSteps[self.level][self.step]];
		self.drawNumber(self.targetColorImg, COLOR_VALUE[self.colorSteps[self.level][self.step]]);
		cv2.imshow("Target Color", self.targetColorImg);

	def updateColor(self, event):
		if not self.isDetecting:
			return;
		if self.step == -1:
			return;
		if self.duration == self.max_Duration:
			return;		
		if self.step == len(self.colorSteps[self.level]):
			self.f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "Vocal message:","Well done! Your task is finished! You can land anytime now.")));
			self.player.say("Well done! Your task is finished! You can land anytime now.", True)
			
			#self.isDetecting = False; 
			self.step = -1;
			return;	

		
		if len(self.dt_colors) > 2:
			self.currentCount += 1;
		else:
			self.currentCount = 0;
		if self.currentCount == 1:
			self.f.write(('{0}\t{1}\t{2}\t{3}\n'.format(time.asctime(), "Vocal message:","Stay still on the marker.", self.colorSteps[self.level][self.step])));
			if self.isVocal: self.player.say("Stay still on the marker.", True);
			self.count_offMarker += 1;
			self.curDirection = True;
		if self.currentCount >= 5:
			self.step += 1;
			self.currentCount = 0;
			self.curDirection = False;
			if self.step < len(self.colorSteps[self.level]):
				self.f.write(('{0}\t{1}\t{2}\t{3}\n'.format(time.asctime(), "Vocal message:","Good job. Now go to color ",self.colorSteps[self.level][self.step])));
				#self.targetColorImg = np.zeros((200,200,3), np.uint8);
				self.targetColorImg[:,:] = DISPLAY_COLOR[self.colorSteps[self.level][self.step]];
				self.drawNumber(self.targetColorImg, COLOR_VALUE[self.colorSteps[self.level][self.step]]);
				#cv2.imshow("Target Color", self.targetColorImg);
				if self.isVocal: self.player.say("Good job. Now go to color {}".format(self.colorSteps[self.level][self.step]), True)	
			else:
				self.targetColorImg[:,:] = DISPLAY_COLOR["white"];
				self.drawNumber(self.targetColorImg, -1);
		#print "color before delleting: ", self.dt_colors
		del self.dt_colors[:];
		
	def saveImage(self):

		try:
			self.it +=1;
			self.cv_image = self.bridge.imgmsg_to_cv(self.image, "bgr8")
			imgName = os.path.join(str(self.folder), "inf_" + str(self.it) +".jpg") ;
#			print "image name:", imgName;
			
			
			#cv.WaitKey(1)
			#cv.SaveImage(imgName, self.cv_image) ;
			if self.isDetecting and self.video != None: self.video.write(np.asarray(self.cv_image[:,:]));
			
			#nextDir = self.myLineDetector.detectRedContour(self.cv_image);
			
			#self.calculateDirection(nextDir);
			if (self.it % 3 == 0 and self.step > -1 and self.step < len(self.colorSteps[self.level])):
				self.calculateDirection(self.myLineDetector.detectLine(np.asarray(self.cv_image[:,:])));
				
				if self.colorSteps[self.level][self.step] in self.myLineDetector.getValueInImage(self.cv_image):
					self.dt_colors.append(self.colorSteps[self.level][self.step])
			#cv.WaitKey(0);
		except CvBridgeError, e:
			print e
		return True;
	

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			
			try:			

					# Convert the ROS image into a QImage which we can display
					image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(0,255,0))
							painter.setBrush(QtGui.QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()
					
					self.saveImage();
					small = cv2.resize(np.asarray(self.cv_image[:,:]), (0,0), fx=4, fy=4) 
					cv2.imshow("Parrot Image window", small);
					cv2.imshow("Target Color", self.targetColorImg);
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
			
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))
		#self.f.write(('{0}\t{1}\n'.format(time.asctime(), self.statusMessage)));
		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()


if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	
	display.show()
	
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
