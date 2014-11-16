#!/usr/bin/env python

# The Joystick Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Joystick Control"
# https://github.com/mikehamer/ardrone_tutorials

# This controller implements the base DroneVideoDisplay class, the DroneController class and subscribes to joystick messages

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import time, os, os.path;

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from my_drone_controller import BasicDroneController
from my_drone_video_display import DroneVideoDisplay
from PythonECGServer import GUI_GSR_ECG
from NLastNumberTest import NumberGenerateur
# Import the joystick message
from sensor_msgs.msg import Joy

# Finally the GUI libraries
from PySide import QtCore, QtGui

# define the default mapping between wheel buttons and their corresponding actions
ButtonEmergency = 0
ButtonLand      = 1
ButtonTakeoff   = 2

# define the default mapping between wheel axes and their corresponding directions
AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 3
AxisZ           = 4

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0
pitchDir	= 0.0#pitch direction: 1==forward; -1=backward; 0=nopitch
isRoll		= 0.0
yawVec		= 0.0

# handles the reception of joystick packets
def ReceiveWheelMessage(data):
	global pitchDir
	pitchDir = 0.0
	global isRoll
	isRoll 	= 0.0


	if data.buttons[0]==1:
		rospy.loginfo("Emergency Button Pressed")
		f.write(('{0}\t{1}\n'.format(time.asctime(), "Emergency Button Pressed")));
		controller.SendEmergency()
	elif data.buttons[ButtonLand]==1:
		rospy.loginfo("Land Button Pressed")
		f.write(('{0}\t{1}\n'.format(time.asctime(), "Land Button Pressed")));
		controller.SendLand()
		#myNumGen.setOnHold(True);
		#display.resetStep();
		#display.disableDetecting();
	elif data.buttons[ButtonTakeoff]==1:
		rospy.loginfo("Takeoff Button Pressed")
		f.write(('{0}\t{1}\n'.format(time.asctime(), "Takeoff Button Pressed")));
		controller.SendTakeoff();
		#myNumGen.setOnHold(False);
		#display.resetStep();

	
	elif data.buttons[3]==1:
		#rospy.loginfo("changing cam channel")
		f.write(('{0}\t{1}\n'.format(time.asctime(), "change cam channel")));
		controller.ChangeCamChannel();
	elif data.buttons[1]==1:	
		f.write(('{0}\t{1}\n'.format(time.asctime(), "toggle detecting")));
		display.toggleDetecting();

	elif data.buttons[2]==1:	
		rospy.loginfo("toggle GSR")
		f.write(('{0}\t{1}\n'.format(time.asctime(), "toggle GSR")));
		gui.toggleGSRRecording();	

	#add button to toggle level
	elif data.buttons[8]==1:	
		rospy.loginfo("toggle level")
		f.write(('{0}\t{1}\n'.format(time.asctime(), "toggle level")));
		display.toggleLevel();	

	#add button to toggle vocal comments
	elif data.buttons[9]==1:	
		rospy.loginfo("toggle vocal comments")
		f.write(('{0}\t{1}\n'.format(time.asctime(), "toggle vocal comments")));
		display.toggleVocal();	
		
			
			
	else:


		if data.buttons[12]==1 or data.buttons[14]==1 or data.buttons[16]==1:
			#rospy.loginfo("pitching forward")
			f.write(('{0}\t{1}\n'.format(time.asctime(), "pitching forward")));
			pitchDir+=1.0;
		elif data.buttons[13]==1 or data.buttons[15]==1 or data.buttons[17]==1:
			#rospy.loginfo("pitching backward")
			f.write(('{0}\t{1}\n'.format(time.asctime(), "pitching backward")));
			pitchDir+=-1.0;

		if data.buttons[4]==1:
			#rospy.loginfo("rolling on the left")
			f.write(('{0}\t{1}\n'.format(time.asctime(), "rolling on the left")));
			isRoll+= -1.0;#data.axes[AxisPitch];
		elif data.buttons[5]==1:
			#rospy.loginfo("rolling on the right")
			f.write(('{0}\t{1}\n'.format(time.asctime(), "rolling on the right")));
			isRoll+=1.0;#data.axes[AxisPitch];
	
		ProcessWheelMessage(isRoll,data.axes[AxisPitch],data.axes[AxisYaw],data.axes[AxisZ]);

#=================================================================
#parameters' names are chosen to be in consistence with the controller.SetCommand()
#called in the function ReceiveJoystickMessage above.
#Added by Ha Dang 27th September 2013.
#
def ProcessWheelMessage(AxRoll, AxPitch, AxYaw, AxZ):
	global pitchDir
	global isRoll
	global yawVec
	#rospy.loginfo("recv: roll {}, pitch {}, yaw {}, z {}".format(AxRoll,AxPitch,AxYaw,AxZ))
	#turn left or right
	if (AxYaw > 0.0):
		AxYaw = 1.0;
	if (AxYaw < 0.0):
		AxYaw = -1.0;

	#move forward or backward
	if (AxPitch > 0.0):
		AxPitch = pitchDir;# * (AxPitch + 1) / 2;
	else:
		AxPitch = 0.0;

	#change in height (higher or lower)
	if (AxZ > 0.0):
		AxZ = pitchDir;# * (AxPitch + 1) / 2;
	else:
		AxZ = 0.0;

	#rolling left or right
	#if (isRoll < 0):
	#	AxRoll	= isRoll;
	#	AxYaw = 0.0;

	#if (isRoll > 0):
	#	AxRoll	= isRoll;
	#	AxYaw = 0.0;

	#rospy.loginfo("setCmd: roll {}, pitch {}, yaw {}, z {}".format(AxRoll/ScaleRoll,AxPitch/ScalePitch,AxYaw/ScaleYaw,AxZ/ScaleZ))
	f.write(('{0}\t{1}\n'.format(time.asctime(), "setCmd; roll: {}, pitch: {}, yaw: {}, z: {}".format(AxRoll/ScaleRoll,AxPitch/ScalePitch,AxYaw/ScaleYaw,AxZ/ScaleZ))));
	controller.SetCommand(AxRoll/ScaleRoll,AxPitch/ScalePitch,AxYaw/ScaleYaw,AxZ/ScaleZ);
	pass;

#=================================================================

def createFolder():
	MainFolder = os.getcwd() + "/logitek_drone_record"
        if os.path.exists(MainFolder) == False:
            	os.mkdir(MainFolder) ;
	folder = "{}/{}".format(MainFolder, time.asctime()) ;
	os.mkdir(folder);
	print "curr folder: ", os.path.abspath(folder)

	return folder;
# Setup the application
if __name__=='__main__':
	import sys
	import subprocess
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_logitekwheel_controller')

	# Next load in the parameters from the launch-file
	ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
	ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
	ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
	AxisRoll        = int (   rospy.get_param("~AxisRoll",AxisRoll) )
	AxisPitch       = int (   rospy.get_param("~AxisPitch",AxisPitch) )
	AxisYaw         = int (   rospy.get_param("~AxisYaw",AxisYaw) )
	AxisZ           = int (   rospy.get_param("~AxisZ",AxisZ) )
	ScaleRoll       = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
	ScalePitch      = float ( rospy.get_param("~ScalePitch",ScalePitch) )
	ScaleYaw        = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
	ScaleZ          = float ( rospy.get_param("~ScaleZ",ScaleZ) )
	
	#create the record folder
	fold = createFolder();
	filename = str(fold) + "/Record_WheelController_{0}.txt".format(time.asctime())
	f = open(filename, "w")
	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	#myNumGen = NumberGenerateur(10,fold);
	display = DroneVideoDisplay(fold)
	controller = BasicDroneController()
	
	#to activate the java GUI.
	#proce = subprocess.Popen('gnome-terminal -e /home/meka/ros_workspace/src/ardrone_tutorials/src/GUI_GSR_ECG.jar', shell=True)
	#subprocess.call(["/home/meka/ros_workspace/src/ardrone_tutorials/src/GUI_GSR_ECG.jar"], shell=True)
	
	
	#gui = GUI_GSR_ECG(None, fold)
    	#gui.title("my GUI_GSR_ECG")
    	#gui.mainloop()		
	


	rospy.Timer(rospy.Duration(1.0), display.updateColor)#check marker
	rospy.Timer(rospy.Duration(1.0), display.updateDir)#check on_track
	rospy.Timer(rospy.Duration(59.0), display.updateDuration)#update duration

	# subscribe to the /joy topic and handle messages of type Joy with the function ReceiveJoystickMessage
	subJoystick = rospy.Subscriber('/joy', Joy, ReceiveWheelMessage)
	
	# executes the QT application
	display.show()
	
	status = app.exec_()
	print "status: ", status;
	f.write(('{0}\t{1}\t{2}\n'.format(time.asctime(), "exit with status ", status)));
	f.close()
	display.closeRecordFile();
	#myNumGen.stop()
	#proce.kill();

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
