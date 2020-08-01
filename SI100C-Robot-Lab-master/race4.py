#!/usr/bin/env python

import rospy
import math
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from cone_msgs.msg import ConeArray
from cone_msgs.msg import Cone
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import Led

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as P

pi2 = math.pi * 2
pi_2 = math.pi / 2

# a class representing a 2D point
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # calculates the geometric distance to some other point
    def geom_dist(self, other):
        return math.sqrt((self.x-other.x) ** 2 + (self.y-other.y) ** 2)
    # calculate the radius of the vector from self to some other point, return the angle
    def angle(self, other):
        angle = math.acos((other.x - self.x) / self.geom_dist(other))
        if other.y < self.y:
            return pi2 - angle
        else:
            return angle

    def centerVec(self, other):
        return Vec((self.x + other.x)/2, (self.y + other.y)/2, self.angle(other) + pi_2)


# a class representing a 2D point with a unit vector
class Vec (Point):
    def __init__(self, x, y, v):
        Point.__init__(self, x, y)
        self.v = v

    # calculate the included angle, return angular
    def includedAngle(self, other):
        return other.v - self.v
    # calculate the linear equation, return three float
    def line(self):
        if math.pi / 4 < self.v % math.pi < math.pi * 3 / 4:
            a = -1
            b = 1 / math.tan(self.v)
            c = self.x - b * self.y
        else:
            a = math.tan(self.v)
            b = -1
            c = self.y - a * self.x
        return a, b, c
    # calculate the intersection of self and other vect, return Point
    def intersection(self, other):
        a1, b1, c1 = self.line()
        a2, b2, c2 = other.line()
        D = a1 * b2 - b1 * a2
        Dx = b1 * c2 - c1 * b2 
        Dy = c1 * a2 - a1 * c2 
        return Point (Dx / D , Dy / D)

    def pointToThisLine(self, other):
        a, b, c = self.line()
        return abs(a * other.x + b * other.y + c) / math.sqrt(a ** 2 + b ** 2)

class Info:
    DEFAULT = 0
    WAIT_CMD = 1
    NO_DATA = 2
    BUMPED = 3
    RUNNING = 4

class Race:
    def __init__(self):
        # messages received for odometry and the cone map are saved in those variables
        self.odom = None
        self.currentPosition = None
        self.cones = []
        self.redCones = []
        self.blueCones = []
        self.bump = None
        self.button = None

        # configuration parameters for the program:
        self.maxLinerSpeed = 1.5
        self.maxAngularSpeed = 1.0

        # flags
        self.infoState = Info.DEFAULT

        # initializes ROS, subscribes topics, and creates the publisher for the topics
        rospy.init_node('race')

        rospy.Subscriber("/odom", Odometry, self.callbackOdometry)
        rospy.Subscriber("/cone_map", ConeArray, self.callbackCones)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.callbackBump)
        rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.callbackButton)

        self.publisherTwist = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        self.publisherLed1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size = 10)
        self.publisherLed2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size = 10)

        self.publisherPath = rospy.Publisher('/race_path_visualization', Marker, queue_size = 10)

        # run the main loop
        self.mainLoop()

    # receive messages
    def callbackOdometry(self, odom):
        (roll, pitch, yaw) = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        self.currentPosition = Vec(odom.pose.pose.position.x, odom.pose.pose.position.y, yaw)
        self.odom = odom
    def callbackCones(self, cones):
        red = []
        blue = []
        for cone in cones.cone_array:
            if cone.type == Cone.RED_CONE:
                red.append(cone)
            elif cone.type == Cone.BLUE_CONE:
                blue.append(cone)
        self.redCones = red
        self.blueCones = blue
        self.cones = cones.cone_array
    def callbackBump(self, bump):
        self.bump = bump
    def callbackButton(self, button):
        self.button = button

    # check if the data has published
    def checkCurrentData(self):
        return not (self.odom == None or len(self.cones) == 0)

    # publish how the robot should move
    def move(self, liner, angular):
        twist = Twist()
        twist.linear.x = min (liner, self.maxLinerSpeed)
        if angular > self.maxAngularSpeed:
            twist.angular.z = self.maxAngularSpeed
        elif angular < -self.maxAngularSpeed:
            twist.angular.z = -self.maxAngularSpeed
        else:
            twist.angular.z = angular
        self.publisherTwist.publish(twist)
    # print information to screen
    def info(self, state):
        if self.infoState is not state:
            self.infoState = state
            if state is Info.WAIT_CMD:
                print 'Waiting for command...'
            elif state is Info.NO_DATA:
                print 'I don\'t know where I am...'
            elif state is Info.BUMPED:
                print 'There\'s something...'
            elif state is Info.RUNNING:
                print 'Marching On...'
    # show path in rviz
    def pathVisualization(self, seq):
        mark = Marker()
        mark.header.frame_id = '/odom'
        mark.ns = 'path'
        mark.type = mark.LINE_LIST
        mark.pose.orientation.w = 0.5
        mark.scale.x = 0.1
        mark.color.g = 1.0
        mark.color.a = 1.0
        for point in seq:
            p = P()
            p.x = point.x
            p.y = point.y
            mark.points.append(p)
        self.publisherPath.publish(mark)


    # set the colour of LEDs
    def setLed(self, which, color):
        led = Led()
        led.value = color
        if which is 1:
            self.publisherLed1.publish(led)
        elif which is 2:
            self.publisherLed2.publish(led)

    # the main loop of the controller. Checks if we have current data - if so executes your algorithm
    def mainLoop(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.setLed(1, Led.BLACK)
            self.setLed(2, Led.BLACK)
            if self.button != None :
                if self.button.button is ButtonEvent.Button1 :
                    self.setLed(1, Led.GREEN)
                    break
            self.info(Info.WAIT_CMD)
            rate.sleep()

        while not rospy.is_shutdown():
            if self.checkCurrentData():
                self.drive()
            else:
                self.move(0, 0)
                self.info(Info.NO_DATA)
        rate.sleep()

    def isBumperActivated(self):
        if self.bump != None :
            if self.bump.state is BumperEvent.PRESSED :
                self.setLed(2, Led.RED)
                return True
            elif self.bump.state is BumperEvent.RELEASED :
                self.setLed(2, Led.BLACK)
        return False

    # need to be modified!!!!!
    def coneInfront(self, currentPosition):
        for cone in self.cones:
            if currentPosition.geom_dist(Point(cone.x ,cone.y)) < 0.5 :
                return True

    # control the robot
    def drive(self):
        if self.isBumperActivated():
            self.move(0, 0)
            self.info(Info.BUMPED)
        else :
            # get distance limit data and show it on rviz
            #
            currentPos = self.currentPosition
            distLim, coneType = self.distSup()
            target, vec, position = self.target()
            end = Point(currentPos.x + distLim * math.cos(currentPos.v), currentPos.y + distLim * math.sin(currentPos.v))
            ouput = [currentPos, end]
            if target != None:
                ouput.append (currentPos)
                ouput.append (position)
            self.pathVisualization(ouput)

            # speed data
            #
            linerSpeed = distLim * 0.6
            vecDif = vec - currentPos.v
            angularSpeed = vecDif * 0.2

            if coneType == Cone.BLUE_CONE:
                if angularSpeed > 0:
                    angularSpeed += 0.8
                else:
                    angularSpeed = 0.8
            elif coneType == Cone.RED_CONE:
                if angularSpeed < 0:
                    angularSpeed -= 0.8
                else:
                    angularSpeed = -0.8

            angularSpeed = angularSpeed * 2

            print (vecDif, angularSpeed)

            # robot run
            self.move(linerSpeed, angularSpeed)
            self.info(Info.RUNNING)

    def findConeClosestTo(self, point, coneType):
        coneFound = None
        if coneType is Cone.UNKNOWN:
            cones = self.cones
        elif coneType is Cone.RED_CONE:
            cones = self.redCones
        elif coneType is Cone.BLUE_CONE:
            cones = self.blueCones
        for cone in cones:
            thisDist = point.geom_dist(Point(cone.x ,cone.y))
            if coneFound == None or thisDist < coneFoundDist:
                coneFound = cone
                coneFoundDist = thisDist
        return coneFound, coneFoundDist

    def distSup(self):
        # configuration parameters for the pathnsert
        #
        thresholdDist = 0.3         # the minimum distance with a cone
        defaultDist = 10.0           # the distance to the edge of robot's sight
        angleLim = 0.8               # cones in this angle matters

        currentPos = self.currentPosition
        cones = self.cones

        # find the closest cone get in the path
        #
        coneFound = None
        for cone in cones:
            point = Point(cone.x ,cone.y)
            thisVec = currentPos.angle(point)
            vecDif = (currentPos.v - thisVec) % pi2
            if min(vecDif, pi2 - vecDif) < angleLim:
                thisDist = currentPos.pointToThisLine(point)
                if thisDist < thresholdDist:
                    geomDist = currentPos.geom_dist(point)
                    distToFoot = math.sqrt(geomDist ** 2 - thisDist ** 2)
                    distSup = distToFoot - math.sqrt(thresholdDist ** 2 - thisDist ** 2)
                    if coneFound == None or distSup < distLim:
                        coneFound = point
                        distLim = distSup
                        coneType = cone.type

        if coneFound == None:
            distLim = defaultDist
            coneType = Cone.UNKNOWN

        return distLim, coneType

    def target(self):
        # configuration parameters for the pathnsert
        #
        thresholdDist = 0.3         # the minimum distance with a cone
        angleLim = 0.8               # cones in this angle matters

        currentPos = self.currentPosition
        cones = self.cones
 
        # sort cones by distance
        sortedCones = []
        for cone in cones:
            point = Point(cone.x, cone.y)
            angle = currentPos.angle(point)
            angleDif = (currentPos.v - angle) % pi2
            if  min (angleDif, pi2 - angleDif ) < angleLim:
                dist = currentPos.geom_dist(point)
                scope = math.atan(thresholdDist / dist)
                sortedCones.append([point, dist, angle, scope])
        if len(sortedCones) == 0:
            return None, None, None
        sortedCones = sorted(sortedCones, key = lambda cone: cone[1])

        # delet covered cones
        flag = [True for i in range(len(sortedCones))]
        for a, coneA in enumerate(sortedCones):
            for b, coneB in enumerate(sortedCones):
                if b > a and abs(coneA[2] - coneB[2]) < coneA[3]:
                    flag[b] = False
        #print (flag)
        for i, f in enumerate(flag):
            if f:
                goal = sortedCones[i]
        return goal[2], goal[1], goal[0]

if __name__ == '__main__':
    try:
        race = Race()
    except rospy.ROSInterruptException:
        pass
    finally:
        pass
