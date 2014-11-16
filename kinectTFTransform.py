'''
Created on Jul 24, 2013

@author: meka
'''
# encoding: utf-8


"""Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.
source: https://gist.github.com/omangin/2414166
"""


#import roslib
#roslib.load_manifest('ros_skeleton_recorder')
import rospy
import tf
import math
from tf.transformations import euler_from_quaternion
from threading import Thread
from control.SoundPlayClient import SoundPlayClient

BASE_FRAME = '/T0'
MEKA_HEAD_FRAME='/meka_static_head'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]
LAST = rospy.Duration()

class KinectThread(Thread):

    def __init__(self, name='kinect_listener',beh_look=None,verbose=False, isAlone=True):
        Thread.__init__(self)
        if isAlone:
        
            rospy.init_node(name, anonymous=True)
            
        self.listener = tf.TransformListener()
        self.user = 1
        self.head_target = None
        self.rhand_target = None
        self.roll = None
        self.keepRunning=True
        self.beh_look = beh_look
        self.verbose = verbose
        self.player = SoundPlayClient()

    
    def get_posture(self, user):
        
        """Returns a list of frames constituted by a translation matrix
        and a rotation matrix.

        Raises IndexError when a frame can't be found (which happens if
        the requested user is not calibrated).
        """
        verbose_msg = []
        framesname = self.listener.getFrameStrings()
        verbose_msg.append('Frames from tf: %s'% (framesname))
        #print "Frames from tf: ", framesname
        
        headframes = [f for f in framesname if "head_%d"%(user) in f and "shell" not in f and "meka" not in f]
        verbose_msg.append("found these head frames %s"% (headframes))
        frames = []
        if len(headframes) == 0:
            return;
        
        user_neck_frame = '/neck_%s' % (user)
        user_head_frame = '/head_%s' % (user)
        user_rHand_frame = 'right_hand_%s' % (user)
        for frame in headframes:
            if user_head_frame not in frame:
                continue;
            try:
            
                                
                verbose_msg.append("searching for: %s" % (frame))
                #print "searching for: %s" % (frame)
                trans, rot = self.listener.lookupTransform("%s_%s" % ("neck", user),
                        "%s" % (frame), LAST)
                #print "found head user: ", frame[-1]
                frames.append((trans, rot))
                #to look at the frame
                self.track_frame = frame
                head_to_frame_vector, _ = self.listener.lookupTransform(MEKA_HEAD_FRAME,
                        "%s" % (frame), LAST)
                head_to_neck_vector, _ = self.listener.lookupTransform(MEKA_HEAD_FRAME,
                        "%s" % (user_neck_frame), LAST)
                head_to_rHand_vec, _ = self.listener.lookupTransform(MEKA_HEAD_FRAME,
                        "%s" % (user_rHand_frame), LAST)
                verbose_msg.append("meka to head: %s"% str(head_to_frame_vector))
                verbose_msg.append("meka to neck: %s"% str(head_to_neck_vector))
                verbose_msg.append("meka to rHand: %s"% str(head_to_rHand_vec))
#                 print "meka to head: ", head_to_frame_vector
#                 print "meka to neck: ", head_to_neck_vector
                self.head_target = head_to_frame_vector;
                self.rhand_target = head_to_rHand_vec
                msg = "user head and neck are equal" 
                self.roll = 0;
                v = head_to_frame_vector[1] - head_to_neck_vector[1]
                if v > 0.02:
                    msg = "head is bigger"
                    self.roll = 1
                if v < -0.02:
                    msg = "head is smaller"
                    self.roll = -1
                
                verbose_msg.append(msg)
#                 print msg
#                 self.player.say(msg, True)
#                 self.getEulerFromQua(trans, rot);
                #set meka to look at the frame
                
                #print "meka.bot.csp.set_target_csp_frame(head_to_frame_vector)"
                
                #print "@TO DO: calculate theta2 for mimicking the rolling of the head"
                
            except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            #raise IndexError
                verbose_msg.append("not found %d"% (user))
                #print "not found ", user
        if self.verbose:
            print '\n'.join(verbose_msg)    
        return frames;
        
    
    def getEulerFromQua(self, trans, rot):
        print 'translation: ',trans
        angles = euler_from_quaternion(rot)
        print 'rotation: ',[(180.0/math.pi)*i for i in angles]
        
    def run(self):
        print "kinectThread started"
        while self.keepRunning and not rospy.core.is_shutdown():
            for i in xrange(5):
                
                coor = self.get_posture(i)
#                 print "im updating coor ", coor
                if coor is None:
                    #print "found nothing for user ", i
                    continue;
                
                self.user = i;
                #print self.user
                if self.beh_look is not None:
                    self.beh_look.lookAndBend(self.head_target, self.rhand_target, self.roll)
                break;
            time.sleep(1)
            #rospy.spin()
        self.player.say("out of the thread", False)
        print "kinectThread stopped!"
    def stop(self):
        self.keepRunning=False
        
import time

if __name__ == "__main__":
    #Init the Kinect object
    kin = KinectThread(verbose= True)
    kin.start()
    time.sleep(15);
    kin.stop()
#     k =0;
#     while k < 10:
#         for i in xrange(5):
#             print "next for"
#             print i, kin.get_posture(i)
#              
#              
#         time.sleep(0.5)
#         k+=1;
        
    print "finished!"