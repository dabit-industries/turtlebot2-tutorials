#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy

# Print "Hello!" to terminal
print "Hello!"

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and ROSLOG
rospy.loginfo("Hello ROS!")

