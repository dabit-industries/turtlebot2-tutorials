# -*-
# Copyright/license here
# -*-

"""
ROSPY Example Class
"""

# Import ROS libraries and messages
import rospy

class example_class():
    """
    turtlebot_dabit.rospy_example_class
    This is an example Python class that initalizes a ROS Node and sets up a timer
    """
    def __init__(self):
        # Diagnostic Node name, ex. ...__module__ = 'turtlebot_dabit.rospy_example_class'
        self._node_name = self.__class__.__module__.split('.')[-1]

        # Handle ROS shutdown
        rospy.on_shutdown(self._rospy_on_shutdown)

        # Get ROS Parameters
        self.node_name = rospy.get_param('~node_name', self._node_name)
        self.anonymity = rospy.get_param('~anonymous', False)

        # Initialize ROS
        rospy.init_node(self.node_name, anonymous=self.anonymity)

        # Print init message
        rospy.loginfo("Hello from ROS Node {0}".format(rospy.get_name()))

        # Set a timer to call function _timer_callback every 2 seconds
        rospy.Timer(rospy.Duration(2), self._timer_callback)

    def _rospy_on_shutdown(self):
        try:
            rospy.logwarn("Shutting Down")
        except Exception, e:
            print e

    def _timer_callback(self, event):
        rospy.loginfo("Time Now: {0}".format(rospy.Time.now().secs))

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
    

