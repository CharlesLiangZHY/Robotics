#!/usr/bin/env python
#
# IST Intelligent Machines and Robotics Practice 1
#

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def callback_joy(joy_data):

    # we got a new joystick message
    twist = Twist()

    # according to the joystick set the control parameters
    twist.linear.x =joy_data.axes[1]*1.2
    twist.angular.z =joy_data.axes[2]*1.2

    # publish the new meslssage
    pub.publish(twist)


def joy_controller():

    # initialize the joystick node
    rospy.init_node('joy_controller', anonymous=True)

    global pub
    # publisher for the new twist robot control commands
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # subscribe to the joystick messages
    rospy.Subscriber("joy", Joy, callback_joy)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joy_controller()