#!/usr/bin/env python
#
# IST Intelligent Machines and Robotics Practice 1
#

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

import curses

# set up curses (console input and output system)
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.nodelay(1)
stdscr.keypad(1)

# global variable for steering commands to the robot
twist = Twist()
# global variable to see if we commanded to stop the robot
stop = 0


# Function that reads the keyboard and adjusts the twist message accordingly. 
# This is also responsible for the printout
def readKeys():
    # use the global variables
    global twist
    global stop
    # a counter to count how many interations we did not press any key
    timeOutCounter = 0
    # maximum value of iterations without key press till we stop the robot
    timeOutValue = 500
    # maximum speed of the robot
    maxSpeed = 1.2
    # maximum turning speed of the robot
    maxTurn = 1

    # read a key from the keyboard
    key = stdscr.getch()

    # check which key was pressed (if any)
    if key == curses.KEY_LEFT:
      #print 'left'
      stop = 0
      timeOutCounter = 0
      # we turn around the z-axis (poinint upwards)
      twist.angular.z += 0.2
    elif key == curses.KEY_RIGHT:
      #print 'right'
      stop = 0
      timeOutCounter = 0
      # we turn around the z-axis (poinint upwards)
      twist.angular.z -= 0.2
    elif key == curses.KEY_UP:
      #print 'up'
      stop = 0
      timeOutCounter = 0
      # we drive forward along the x axis
      twist.linear.x += 0.1
    elif key == curses.KEY_DOWN:
      #print 'down'
      stop = 0
      timeOutCounter = 0
      # we drive forward (or backward) along the x axis
      twist.linear.x -= 0.1
    elif key != -1:
      #print 'stop'
      # if any other key was pressed we stop the robot
      stop = 1
      timeOutCounter = 0
      twist.linear.x = 0
      twist.angular.z = 0
    else:
      # key is -1 == no key pressed
      timeOutCounter += 1
      # check if we are over the timeout (500 iterations = 5 seconds)
      if timeOutCounter >= timeOutValue:
        #print 'timeout'
        # this is a timeout - we stop the robot
        twist.linear.x = 0
        twist.angular.z = 0

    # make sure we do not exceed the maximum values!
    if twist.linear.x > maxSpeed:
      twist.linear.x = maxSpeed
    if twist.linear.x < -maxSpeed:
      twist.linear.x = -maxSpeed
    if twist.angular.z > maxTurn:
      twist.angular.z = maxTurn
    if twist.angular.z < -maxTurn:
      twist.angular.z = -maxTurn

    # prepare an informative line of text
    text = "Kobuki forward %f  turn %f " % (twist.linear.x, twist.angular.z)
    if timeOutCounter >= timeOutValue:
      text += "timeout!         "
    elif stop == 1:
      text += "stop!            "
    else:
      text += "                 "
    # print out the text (at position 0, 0)
    # stdscr.addstr(0, 0, text)

# The main loop of our controller
def keyop():
    # Initialize the rosnode with the name 'keyop'
    rospy.init_node('keyop')
    # Create a publisher for publishing (sending) messages on the topic (name) '/mobile_base/commands/velocity'.
    # The messages are of type 'Twist'
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    # Create a Rate object for running a loop with 100 Hz
    rate = rospy.Rate(100) # 100hz

    # As long as this rosnode is running do....
    while not rospy.is_shutdown():

        # read the keyboard input, update the twist message, and print some info
        readKeys()

        # Publish the updated twist message
        pub.publish(twist)
        # Sleep as much time as is needed to achive 100 Hz
        rate.sleep()


def do_measure(o):
  pos = o.pose.pose.position

  stdscr.addstr(0, 0, str((pos.x ** 2 + pos.y ** 2)**0.5))
  # rospy.loginfo("data:", data)


# The main program
if __name__ == '__main__':
    try:
      # Try to make the keyop object - the main loop is running in the constructor

      rospy.Subscriber('/odom', Odometry, do_measure)
      keyop()
      # If there was an exception (for example no roscore running) do ... nothing (and then exit)
    except rospy.ROSInterruptException:
        pass
