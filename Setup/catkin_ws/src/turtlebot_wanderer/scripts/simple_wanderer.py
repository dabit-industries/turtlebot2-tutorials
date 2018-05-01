#!/usr/bin/env python
'''
This is an implementation of the IHeartRobotics Turtlebot Wanderer program that incorporates the bumper and an ultrasonic sensor located on top of the Turtlebot.
'''

import rospy
from kobuki_msgs.msg import SensorState, Sound, BumperEvent, ButtonEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class wanderer:
    
    def __init__(self):
	'''Initializes an object of this class.

	The constructor creates a publisher, a twist message.
	3 integer variables are created to keep track of where obstacles exist.
	3 dictionaries are to keep track of the movement and log messages.'''
	self.msg = Twist()
	self.sect_1 = 0
	self.sect_2 = 0
	self.sect_3 = 0
        self.bumped = False
        self.stopped = False
	self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2}
	self.fwd = {0:.25,1:0,10:0,11:0,100:0,101:0,110:0,111:0}
	self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}


    def listener(self):
        '''Initializes node, creates subscriber, and states callback 
        function.'''
        rospy.init_node('navigation_sensors')
        rospy.loginfo("Subscriber Starting")
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.sub_bump = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.callback_bumper)
        self.sub_button = rospy.Subscriber('/mobile_base/events/button', ButtonEvent, self.callback_button)
        self.pub_sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
	self.pub_navi = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
        rospy.spin()

    def reset_sect(self):
	'''Resets the below variables before each new scan message is read'''
        if self.bumped:
            return
	self.sect_1 = 0
	self.sect_2 = 0
	self.sect_3 = 0

    def sort_bump(self, msg):
        '''
            0             1              2
           left         front          right
        veer right    veer right     veer left
        '''
        if msg.state:
            self.bumped = True
            self.bumper = msg.bumper
            self.sect_1 = 1
            self.sect_2 = 1
            self.sect_3 = 1
        else:
            self.bumped = False
        rospy.loginfo(msg)


    def sort_scan(self, laserscan):
	'''Goes through 'ranges' array in laserscan message and determines 
	where obstacles are located. The class variables sect_1, sect_2, 
	and sect_3 are updated as either '0' (no obstacles within 0.7 m)
	or '1' (obstacles within 0.7 m)

	Parameter laserscan is a laserscan message.'''
        if self.bumped:
            return
	entries = len(laserscan.ranges)
	for entry in range(0,entries):
	    if 0.4 < laserscan.ranges[entry] < 0.75:
		self.sect_1 = 1 if (0 < entry < entries/3) else 0 
		self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
		self.sect_3 = 1 if (entries/2 < entry < entries) else 0
	rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3))

    def movement(self, sect1, sect2, sect3):
	'''Uses the information known about the obstacles to move robot.

	Parameters are class variables and are used to assign a value to
	variable sect and then	set the appropriate angular and linear 
	velocities, and log messages.
	These are published and the sect variables are reset.'''
	sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
	rospy.loginfo("Sect = " + str(sect)) 
	
        if self.stopped:
            self.msg.angular.z = 0
            self.msg.linear.x = 0
            self.pub_sound.publish(0)
        else:
            self.msg.angular.z = self.ang[sect]
            self.msg.linear.x = self.fwd[sect]
	rospy.loginfo(self.dbgmsg[sect])
	self.pub_navi.publish(self.msg)

	self.reset_sect()

    def callback_laser(self, msg):
        self.sort_scan(msg)
        self._callback(msg)

    def callback_bumper(self, msg):
        self.sort_bump(msg)

    def callback_button(self, msg):
        if msg.state:
            self.stopped = True
        else:
            self.stopped = False

    def _callback(self,msg):
	'''Passes laserscan onto function sort which gives the sect 
	variables the proper values.  Then the movement function is run 
	with the class sect variables as parameters.

	Parameter laserscan is received from callback function.'''
	self.movement(self.sect_1, self.sect_2, self.sect_3)
	
if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = wanderer()
    sub_obj.listener()

