#!/usr/bin/env python

#import Leap, sys
#from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import Tkinter as tk


# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('leap')
import rospy
from geometry_msgs.msg import Vector3
from leap.msg import leapros

from drone_controller import BasicDroneController

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

import sys
# Firstly we setup a ros node, so that we can communicate with the other packages
rospy.init_node('ardrone_LeapMotion_controller')
#droneController = BasicDroneController()

class LeapController(object):
    def __init__(self):
	self.pitch = 0
	self.roll = 0
	self.yaw_velocity = 0 
	self.z_velocity = 0
	self.camId = 0;
	self.z_pos = 100;
        self.drone_cmd = 0#ready to take off
	self.noDrone = True;

	self.subNavdata = rospy.Subscriber('leapmotion/data',leapros,self.ReceiveNavdata) 
	print "Initialized"

    def ReceiveNavdata(self, data):

	nfingers = data.nfingers;
	print "Drone_cmd {}, nfingers: {}".format(self.drone_cmd, nfingers);
	if self.drone_cmd == 0 and nfingers > 2:
	    print "#Begin taking off, do not want any driving command"
	    self.drone_cmd = 1;#Taking off, do not want any driving command
	    #self.z_pos = data.palmpos.y;
	    if self.noDrone:
		print "##droneController.SendTakeoff()"
	    return;
	if self.drone_cmd == 1 and nfingers > 1:
	    print "#Still taking off, do not want any driving command"
	    #self.drone_cmd = 2;#Done taking off, ready to receive driving commands
	    #self.z_pos = data.palmpos.y;
	    return;
	if self.drone_cmd == 1 and nfingers < 2:
	    print "#Done taking off, ready to change state to receive commands"
	    self.drone_cmd = 2;#Done taking off, ready to receive driving commands
	    #self.z_pos = data.palmpos.y;
	    return;
	if self.drone_cmd == 2 and nfingers > 2:
	    print "#Ready to receive driving commands"
	    self.drone_cmd = 3;#Done taking off, ready to receive driving commands
	    self.z_pos = data.palmpos.y;
	    return;

	if self.drone_cmd != 0 and nfingers < 1:
	    self.drone_cmd = 0;
	    if self.noDrone:
		print "##droneController.SendLand()"
	    return;

	if self.drone_cmd == 0:
	    print "nothing to send because the drone is off"
	    return;

	# Now we handle moving
	self.yaw_velocity = -data.ypr.x;

	self.pitch = -data.ypr.y;
		
	self.roll = data.ypr.z;

	if (data.palmpos.y > self.z_pos + 20):
	    print "hand moves up";
	    self.z_pos = data.palmpos.y;
	    self.z_velocity += 1
	if (data.palmpos.y < self.z_pos - 20):
	    print "hand moves down";
	    self.z_pos = data.palmpos.y;
	    self.z_velocity += -1

	# finally we set the command to be sent. The controller handles sending this at regular intervals
	print (self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
	if not self.noDrone:
		droneController.SetCommand(self.roll, self.pitch, self.yaw_velocity, 0)


root = tk.Tk()
def checkKeyStroke(event):
	#while (1):
	    # Keep this process running until Enter is pressed
	    #print "Press a to quit..."
	    #ch = sys.stdin.readline()
	    x = event.char
	    if event.keysym == 'Escape':
		
		root.destroy()
		return;
	    if x == 'e':
		print "droneController.SendEmergency()"
		return;
	    if x == 'y':
		print "droneController.SendTakeoff()"
		return;
	    if x == 'h':
		print "droneController.SendLand()"
		return;

	    else:
		print "not an Escape", x;
		return;
    
    
def main():
    


    # Create a sample listener and controller
    leapController = LeapController()

    root.bind_all('<Key>', checkKeyStroke)
    #root.withdraw();
    root.mainloop()
  
    
    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit(0)

if __name__ == "__main__":
    main()
