# -*- coding: utf-8 -*-
'''
-------------------------------------------------------------------------------
Copyright ©  2019  Ali M. Al-Bayaty

TurtleBot Signals Detector is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

TurtleBot Signals Detector is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
-------------------------------------------------------------------------------

TurtleBot Signals Detector
@author: Ali Al-Bayaty
Instructor: Marek Perkowski, Ph.D.
Portland State University

Intelligent Robotics I: Project 1 (TurtleBot Signals Detector) using Gazebo.
Website: <https://github.com/albayaty/gazebo-turtlebot>

Created on Sun Nov  3 14:12:15 2019
-------------------------------------------------------------------------------
'''

#-----------------------------------------------------------------------------
# Loading the required libraries:
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
# --------------------------------------------------------------------------

# Global settings:
# Lower and upper bands of Red object (Stop Signal):
lower_r = np.array([150,100,100])
upper_r = np.array([179,255,255])
# Lower and upper bands of Yellow object (Turn-right Signal):
lower_y = np.array([20,100,100])
upper_y = np.array([40,255,255])
# Lower and upper bands of Green object (Forward Signal):
lower_g = np.array([50,100,100])
upper_g = np.array([70,255,255])
# Lower and upper bands of Blue object (Turn-left Signal):
lower_b = np.array([110,100,100])
upper_b = np.array([130,255,255])
# Forward speed at M/s:
speed = 0.2
# Angular rotation at Radians/s:
rotate= 2.4 
# Signal's area/region:
signal_area = 100000
# Counting the movement steps:
counter = 0
# For messaging purpose:
message = True
# --------------------------------------------------------------------------

# The signal detection function:
def signal_detector():    
    # ROS Subscriber for the Camera/Image topic:    
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback)

    # spin() loops signal_detector() until this node is terminated:
    rospy.spin()
# --------------------------------------------------------------------------
    
# The procedural callback function of signal_detector():
def callback(data):
    global counter
    global message
    
    if( message == True ):
        print(">> Done!")
        print(" ")
        print(">> Starting TurtleBot")
        message = False
 
    # Bridging between ROS images and OpenCV2 images:
    bridge = CvBridge()

    # Fetching the image(s) from CvBridge() and converting to HSV colorspace:
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)    
    
    # Masking HSV colorspace with its lower and upper levels of the same color:
    mask_g = cv.inRange(hsv_frame, lower_g, upper_g)
    mask_y = cv.inRange(hsv_frame, lower_y, upper_y)
    mask_b = cv.inRange(hsv_frame, lower_b, upper_b)
    mask_r = cv.inRange(hsv_frame, lower_r, upper_r)
    
    # Comparing the original fetched image(s) with the masked ones through bitwise ANDing:
    bitwise_g = cv.bitwise_and(frame,frame, mask= mask_g)
    bitwise_y = cv.bitwise_and(frame,frame, mask= mask_y)
    bitwise_b = cv.bitwise_and(frame,frame, mask= mask_b)
    bitwise_r = cv.bitwise_and(frame,frame, mask= mask_r)
    
    # Finding the contours of signals/masks:
    contours_g, hierarchy_g = cv.findContours(mask_g, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_y, hierarchy_y = cv.findContours(mask_y, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_b, hierarchy_b = cv.findContours(mask_b, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_r, hierarchy_r = cv.findContours(mask_r, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
    # Drawing the found contours and finding their areas (Moments):
    try:
        cv.drawContours(bitwise_g, contours_g, -1, (255,255,255), 3)
        area_g = cv.contourArea(contours_g[0])
    except IndexError:
        area_g = 0.0
    try:
        cv.drawContours(bitwise_y, contours_y, -1, (255,255,255), 3)
        area_y = cv.contourArea(contours_y[0])
    except IndexError:
        area_y = 0.0
    try:
        cv.drawContours(bitwise_b, contours_b, -1, (255,255,255), 3)
        area_b = cv.contourArea(contours_b[0])
    except IndexError:
        area_b = 0.0
    try:
        cv.drawContours(bitwise_r, contours_r, -1, (255,255,255), 3)
        area_r = cv.contourArea(contours_r[0])
    except IndexError:
        area_r = 0.0
    
    # Finding the index of the maximum detected signal/object: 
    areas = [area_g, area_y, area_b, area_r]
    index_max_area = areas.index(max(areas))

    # Viewers:
    cv.imshow('< TurtleBot Camera >', frame)
    cv.imshow('< Green Signal Detection >', bitwise_g)
    cv.imshow('< Yellow Signal Detection >', bitwise_y)
    cv.imshow('< Blue Signal Detection >', bitwise_b)
    cv.imshow('< Red Signal Detection >', bitwise_r)
    
    # Continuous forward movement of TurtleBot:
    # ROS publisher that tells TurtleBot to move:
    go = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    # TurtleBot will stop, if a user don't keep telling it to move. How often to move? 10 HZ:
    rate = rospy.Rate(10);
    # Velocity datatype (Default Twist() has linear.x=0 and angular.z=0):
    move_cmd = Twist()
    
    # Forward movement at (speed) rate of M/s:
    move_cmd.linear.x = speed
    # Turning at 0 Radians/s:
    move_cmd.angular.z = 0
    # Publishing the velocity:
    go.publish(move_cmd)
    # Waiting for 0.1 seconds (10 HZ) and publishing again:
    rate.sleep()
    counter += 1

    # Green signal detected:
    #if( index_max_area == 0 and areas[index_max_area] > signal_area ):
        #print('>> Green signal detected ...')
    
    # Yellow signal detected, i.e. turning right:
    if( index_max_area == 1 and areas[index_max_area] > signal_area and counter > 7 ):
        counter = 0
        print('>> Yellow signal detected, turning right')
        go.publish(Twist())
        rate.sleep()
        #rospy.sleep(1)
        # Forward movement at (speed) rate of M/s:
        move_cmd.linear.x = speed
        # Turning at (-pi) Radians/s:
        move_cmd.angular.z = -1*rotate
        # Publish the velocity:
        go.publish(move_cmd)
        # Waiting for 0.1 seconds (10 HZ) and publishing again:
        rate.sleep()
        rospy.sleep(1)
            
    # Blue signal detected, i.e. turning left:
    elif( index_max_area == 2 and areas[index_max_area] > signal_area and counter > 7 ):
        counter = 0
        print('>> Blue signal detected, turning left')
        go.publish(Twist())
        rate.sleep()
        #rospy.sleep(1)
        # Forward movement at (speed) rate of M/s:
        move_cmd.linear.x = speed
        # Turning at (pi) Radians/s:
        move_cmd.angular.z = rotate
        # Publishing the velocity:
        go.publish(move_cmd)
        # Waiting for 0.1 seconds (10 HZ) and publishing again:
        rate.sleep()
        rospy.sleep(1)
            
    # Red signal detected, i.e. stop:
    elif( index_max_area == 3 and areas[index_max_area] > signal_area/1.5 ):
        print('>> Red signal detected, stoping ...')
        go.publish(Twist())
        rate.sleep()
        rospy.sleep(1)
        shutdown()    
# --------------------------------------------------------------------------

def shutdown():
    # Closing all opened OpenCV2 windows:
    cv.destroyAllWindows()    
    rospy.loginfo("ROS Node is terminated!")
    #rospy.signal_shutdown('SHUTDOWN_SIGNAL')    
# --------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        # Names of the OpenCV2 windows, as Viewers:
        cv.startWindowThread()        
        cv.namedWindow('< TurtleBot Camera >')
        cv.namedWindow('< Green Signal Detection >')
        cv.namedWindow('< Yellow Signal Detection >')
        cv.namedWindow('< Blue Signal Detection >')
        cv.namedWindow('< Red Signal Detection >')
        # Message to the user:
        print(" ")
        print("-----------------------------------------")
        print("  Project 1: TurtleBot Signals Detector")
        print("-----------------------------------------")
        print(" ")
        print("NOTE: To exit this program, press CTRL + C")
        print(" ")
        key = raw_input("Type the letter 'g' to start TurtleBot: ")

        #while (not rospy.is_shutdown() and key != 'g'):
        while (key != 'g'):
            key = raw_input("Type the letter 'g' to start Turtlebot: ")

        print(" ")
        print(">> Initializations, please wait ...")
        # ROS Initializations:
        rospy.init_node('turtlebot', anonymous=False)
        rospy.on_shutdown(shutdown)
        # Calling the implementation function:
        signal_detector()
        
    except rospy.ROSInterruptException: pass
# --------------------------------------------------------------------------
