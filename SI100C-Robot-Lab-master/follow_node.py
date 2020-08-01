#!/usr/bin/env python

# Importing RosPy
import rospy
# Importing the Image topic
from sensor_msgs.msg import Image
# Importing the Twist message to control the robot
from geometry_msgs.msg import Twist

# Importing numpy for the image array
import numpy as np

# Importing sys for the command line arguments
import sys


# Importing the module for the canny edge detector you are implementing for HW 1
import canny_detector
# Importing the module for the ransanc circle detector you are implementing for HW 2
import ransac_circle_detector
# Importing the module for the hough square detector you are implementing for HW 3
import hough_square_detector

from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound


""" 
  The FollowNode Class

  Initializes ROS in the constructor.
  
  The main work is done in the callbackImage function that is called for every image.
  
  The helper function crateImage creates a publishable image from the array of 
  values for debugging and visualization.

"""
class FollowNode:
 

    # The variables for the debug image publishers. Can only be created after the ROS node is initialized.
    pub_image_edge = None
    pub_image_circle = None
    pub_image_square = None

    # Booleans controlling if the debug images are being published
    publish_edge = False
    publish_circle = False
    publish_square = False

    
    # The twist message for controlling the robot
    pub_twist = None
    pub_led = None
    pub_sound = None

    button = ButtonEvent()

    bumper = BumperEvent()


    """
        Creates a ROS Image from the array. 
        
        Pixels with value 1 are painted red (for the circle).
        Pixels with value 2 are painted green (inliers). 
        All other pixels are painted according to their value.
    
    """
    def createImage(self, arr, image, width, height):
        image.width = width
        image.height = height
        image.step = image.width * 3
        image.encoding = 'bgr8'

        outp = np.zeros(image.width * image.height * 3, dtype=np.int8)

        county = 0
        for pixel in arr:
            # Circle center cross lines
            # red
            if pixel == 1:
                outp[county] = 0
                outp[county + 1] = 0
                outp[county + 2] = 255
            # Inliers
            # green
            elif pixel == 2:#inliers
                outp[county] = 0
                outp[county + 1] = 255
                outp[county + 2] = 0
            # blue
            elif pixel == 3:#inliers
                outp[county] = 255
                outp[county + 1] = 0
                outp[county + 2] = 0
            # yellow
            elif pixel == 4:
                outp[county] = 0
                outp[county + 1] = 255
                outp[county + 2] = 255
            # Output mono image
            else:# output mono image
                outp[county] = pixel
                outp[county + 1] = pixel
                outp[county + 2] = pixel

            county += 3

        image.data = outp.tostring()


    """
      The callbackImage function.
      
      Gets called for every new image that arrives. 
      This function creates an intensity (grey-scale) image from the RGB data.
      Then your functions are called:
      
      canny_detector.canny
      ransac_circle_detector.ransac_cicle
      hough_scquare_detector.hough_square
      
      Debug images are created for all three functions.
    
    """
    def callbackImage(self, image):

        if image.encoding != "bgr8":
            rospy.loginfo("Image has the wrong format! Should be rgb8, but is %s", image.encoding)
            return

        #print image.header

        # Create a numpy array from the image data
        np_arr = np.fromstring(image.data, np.uint8)

        # Create another numpy array with the grey-scale data
        arr = (np_arr.reshape((image.width * image.height, 3)).sum(axis=1)) / 3. # convert to gray image

        # Create the numpy array that will hold the result
        result = np.zeros(image.width * image.height, dtype=np.uint8)

        # Call your canny edge detector with a threashold of 175
        canny_detector.canny(arr, result, image.width, image.height, 175)

        if self.publish_edge:
            # Publish the debug image for the canny edge detector
            image_canny = Image()
            self.createImage(result, image_canny, image.width, image.height)
            self.pub_image_edge.publish(image_canny)

        # Create the result variables for the best detected circle
        circle_x = 0
        circle_y = 0
        circle_radius = 0
        
        # Call your cricle detector
        circle_x, circle_y, circle_radius = ransac_circle_detector.ransac_cicle(result, image.width, image.height)

        if self.publish_circle:
            # Publish the debug image for the circle 
            image_circle = Image()
            self.createImage(result, image_circle, image.width, image.height)
            self.pub_image_circle.publish(image_circle)

        
        # Invoke your control function
        self.controlRobot_circle(circle_x, circle_y, circle_radius)
        
        
        # In order to really make sure that only the latest image gets processed you can uncomment the next two lines... 

        #self.subscriber.unregister()
        #self.subscriber = rospy.Subscriber("/camera/rgb/image_color", Image, self.callbackImage, queue_size=1, buff_size=4000000)


    
        square_x = 0
        square_y = 0 
        square_radius = 0 
        square_angle = 0
        
        # Call your square detector
        square_x, square_y, square_radius, square_angle = hough_square_detector.hough_square(result, image.width, image.height)

        if self.publish_square:
            # Publish the debug image for the square 
            image_square = Image()
            self.createImage(result, image_square, image.width, image.height)
            self.pub_image_square.publish(image_square)

        
        # Invoke your control function
        self.controlRobot_square(square_x, square_y, square_radius, square_angle)

    def callbackBump(self,bumper):
        self.bumper = bumper

    def callbackButton(self,button):
        self.button = button

    def controlRobot_square(self, square_x, square_y, square_radius, square_angle):
        twist = Twist()
        # fill your control code here...
        led = Led()
        sound = Sound()
        led.value = Led.BLACK

        if self.button.button is ButtonEvent.Button2 :
            if square_radius == 0:
                twist.linear.x = 0
                twist.angular.z = 0
            else:
                led.value = Led.RED

                if square_x < 320:
                   twist.angular.z = -0.5

                if square_x > 320:
                    twist.angular.z = 0.5


                if square_radius <= 150:
                   twist.linear.x = 0.3

                elif square_radius > 150:
                   twist.linear.x = 0

        else:
            pass
           
        if self.bumper.state is BumperEvent.PRESSED:
            twist.linear.x = 0
            twist.angular.z = 0
            sound.value = 4
       
        self.pub_led.publish(led)
        self.pub_sound.publish(sound)
        self.pub_twist.publish(twist)

    def controlRobot_circle(self, circle_x, circle_y, circle_radius):
        twist = Twist()
        # fill your control code here...
        led = Led()
        sound = Sound()
        led.value = Led.BLACK

        if self.button.button is ButtonEvent.Button1 :
            if circle_radius == 0:
                twist.linear.x = 0
                twist.angular.z = 0
            else:
                led.value = Led.GREEN

                if circle_x < 320:
                   twist.angular.z = 0.5

                if circle_x > 320:
                    twist.angular.z = -0.5
 

                if circle_radius <= 150:
                    twist.linear.x = 0.3

                elif circle_radius > 150:
                    twist.linear.x = 0

        else:
            pass
           
        if self.bumper.state is BumperEvent.PRESSED:
            twist.linear.x = 0
            twist.angular.z = 0
            sound.value = 4
       
        self.pub_led.publish(led)
        self.pub_sound.publish(sound)
        self.pub_twist.publish(twist)


    """
        The constructor for the FollowNode
    """
    def __init__(self):

        # Read the arguments from the command line...
        if len(sys.argv) != 4:
            print "Arguments: publish_edge(1/0)  publish_circle(1/0)  publish_square(1/0)"
            return
        
        self.publish_edge = int(sys.argv[1])
        self.publish_circle = int(sys.argv[2])
        self.publish_square = int(sys.argv[3])


        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('ros_wrapper', anonymous=True)

        # Subscribe to the color image topic and callback the callbackImage function. 
        # Set the queue size to one and the (total) buffer size fo 4M => only get the latest image.
        # Seems not to work though :/ - see the work around at the end of the callback function...
        self.subscriber = rospy.Subscriber("/camera/rgb/image_color", Image, self.callbackImage, queue_size=1, buff_size=40000000)
        self.subscriber = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.callbackBump)
        self.subscriber = rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.callbackButton)

        # The publishers for the debug images
        self.pub_image_edge   = rospy.Publisher('/debug_image_edge', Image, queue_size=1)
        self.pub_image_circle = rospy.Publisher('/debug_image_circle', Image, queue_size=1)
        self.pub_image_square = rospy.Publisher('/debug_image_square', Image, queue_size=1)

        self.pub_twist = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.pub_led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size = 10)
        self.pub_sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size = 10)


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


# The main script function
if __name__ == '__main__':
    # Create an instance of the  FollowNode class (called follower). Will invoke the constructor (__init__) of the FollowNode class.
    follower = FollowNode()
