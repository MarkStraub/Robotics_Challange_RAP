#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Moving():
    '''Class for reacting to aruco_tags.'''

    def __init__(self):
        self.publisher = rospy.Publisher('/summit_xl/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Initialize Moving Class")


    def rotate(self, z):        
        msg = Twist()
        msg.angular.z = z
        self.publisher.publish(msg)