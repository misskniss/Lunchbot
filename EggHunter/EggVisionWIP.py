#!/usr/bin/env python
""" 
  Author: Kevin Jin 
  Contributers: Jen Kniss, Hayden Mills
  Date: Sprint 2016
  Purpose: For use with vision/kinect functionality of the Egg Hunter 
           research project at Boise State University
  Example code of how to convert ROS images to OpenCV's cv::Mat

  See also cv_bridge tutorials: 
    http://www.ros.org/wiki/cv_bridge
    roslaunch turtlebot_bringup
    roslaunch openni_launch openni.launch
    rosrun image_viewe image.viewer
"""
import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from math import atan, pow, sqrt, sin, cos
xpx=0
ypx=0
px=0.0

class EggVisionWIP():
    
    def __init__(self):
        # initialize a node called hw2
        #rospy.init_node("hw2")
        # create a window to display results in
        cv2.namedWindow("Keypoints", 1)
        rospy.Subscriber("camera/rgb/image_color", Image, self.callback1)
        rospy.Subscriber("camera/depth/image", Image, self.callback2)
        print("is this running?")
    
    def callback2(self, data):
        global xpx
        global ypx
        global px
        bridge=CvBridge()
        cv_imagepp = bridge.imgmsg_to_cv2(data)
        cv_image = cv_imagepp[288:480,0:640]
        if(xpx!=0):   
            px=cv_image[ypx,xpx]
    
    def callback1(self, data):
        global xpx
        global ypx
    	global px
    	bridge=CvBridge()
    	cv_imagepp = bridge.imgmsg_to_cv2(data)
    	cv_imagep = cv_imagepp[288:480,0:640]   
    	params = cv2.SimpleBlobDetector_Params()
    	params.filterByConvexity = True
    	params.minConvexity = .73
    	params.filterByArea = True
    	params.minArea = 30 #should this be higher?
    	params.filterByInertia = True
    	params.minInertiaRatio = .1
    	params.maxInertiaRatio = 1
    	params.minThreshold = 110
	params.filterByCircularity = True
    	params.minCircularity = .4
    	ver = (cv2.__version__).split('.')
    	
        if int(ver[0]) < 3:
            detector = cv2.SimpleBlobDetector(params)
    	else:
	    detector = cv2.SimpleBlobDetector_create(params)
    	
        cv_imaged = cv2.GaussianBlur(cv_imagep,(5,5),0)
    	cv_imageg = cv2. cvtColor(cv_imaged, cv2.COLOR_BGR2GRAY)
    	detector=cv2.SimpleBlobDetector()    
    	canny_img = cv2.Canny(cv_imaged,110,270)
    	keypoints=detector.detect(canny_img)
    	test_with_keypoints= cv2.drawKeypoints(cv_imagep, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    	cv2.imshow('Keypoints',test_with_keypoints)
    	
        try:
	    xpx=keypoints[0].pt[0]
	    ypx=keypoints[0].pt[1]
    	except IndexError:
	    valuecool=0
	    #print("...")
    	#print("THIS IS THE END %d",self.getDistance())
        cv2.waitKey(5)
    
    def getDistance(self):
        broken=False
    	
        if (px!=0.0 and xpx!=0 and ypx!=0):
    	    flatDepth=sqrt(pow(px,2)-pow(.3,2))
	    magicAngle=3.1416*57*(xpx-320)/115200
            if (magicAngle < 0):
                magicAngle=abs(magicAngle)
	        xdist=-1*flatDepth*sin(magicAngle)
	    else:
	        xdist=flatDepth*sin(magicAngle)
            magicAngle=abs(magicAngle)
	    ydist=flatDepth*cos(magicAngle)
            print type(ydist)
            if (math.isnan(xdist) or math.isnan(ydist)):
                return (-1,-1)
            else:
                return (xdist, ydist)
        else:
	    return (-1,-1)
	    

'''if __name__ == '__main__':
    EggVisionWIP()
    try:  
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()'''


