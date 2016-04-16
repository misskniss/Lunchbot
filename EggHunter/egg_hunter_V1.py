#!/usr/bin/env python

###
# KEVIN: DO NOT CHANGE THIS FILE, make a copy...
###

# Author:   Jen Kniss
# Date:     March 2016
# Purpose:  Controller class for running vision, navigation, and 
#           and serial communication to hunt for eggs on the blue turf

import ros
import roslaunch
import rospy
import roslib
import tf
import argparse
import actionlib
import get_position
import subprocess
import os
import serial
import time
from actionlib_msgs.msg import *
from EggVisionWIP import EggVisionWIP
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from MoveIt import MoveIt
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from serial_test import SerialTest
from sys import executable
from kobuki_msgs.msg import ButtonEvent

''' Controller class for integrating the Vision, Movement, and Serial
    module for the Egg Hunter research Project.

    Notes:
        The Serial program running on the arduino could use a change
        that allows this controller class to tell it what state
        the scoop should set itself to, rather than using the somewhat
        fragile "* to change the state" implementation it is using now. 
'''
class EggHunter():

    def __init__(self, field, basket):

        self.FIELD_EDGE = field
        self.BASKET = basket
        self.MAX = 15.0
        print "Field location set to: " + str(self.FIELD_EDGE)
        print "Basket location set to: " + str(self.BASKET)

        self.kobuki = MoveIt()
        self.position = (0.0,0.0)

        #rospy.init_node('Egg_Hunt', anonymous=False) 
        rospy.on_shutdown(self.shutdown)


    #def set_state(self, value):
    #    self.kobuki.set_state(value)
   
    """ Handles the actual movement to a target
    """
    def go_to(self, target):
        print "going to: " + str(target)
        print type(target)
        #if ((not target[0] == 0.0) and (not target[0] == -0.0)):
        if target:
            self.kobuki.horizontal(target[0])
            self.kobuki.forward(target[1])
            #self.kobuki.horizontal(0.0)
        #if (target[1] != 0 and target[1] != -0):
           # self.kobuki.forward(target[1])
            x_val = self.position[0]
            x_val += target[0]
            y_val = self.position[1]
            y_val += target[1]
            self.position = (x_val, y_val)
            print "my position is now: " + str(self.position)
     
    """ Convenience method for going to the field.
        "Field" in this case just means ' clear the basket ' 
        so we are not detecting the egg we just put down. """
    def go_to_field(self):
        self.go_to_basket()
        print "Heading to field..."
        self.go_to(self.FIELD_EDGE)
        print "At field."

    """ Convenience method for going to the basket/home base. 
        "Basket" in this case means 'where ever she started.' """
    def go_to_basket(self):
        print "Calculating basket location..."
        x_val = self.position[0] * -1
        y_val = self.position[1] * -1
        new_position = (x_val, y_val)
        self.go_to(new_position)

        print "Heading to basket..."
        self.go_to(self.BASKET)
        print "At Basket."
    
    def go_to_edge_from_field(self):
        xtrup=self.FIELD_EDGE[0]-self.position[0]
        ytrup=self.FIELD_EDGE[1]-self.position[1]
        yoloswag=(xtrup,ytrup)
        self.go_to(yoloswag)
    
    """ Uses the vision analysis instance to find
        a set of x,y distances from the center of the scoop in meters
        The center of the scoop on March 25th 2016 was 40 CM from the 
        front of the kinect."""
    def identify(self):
        
        print "Identifying... "
        ev = EggVisionWIP()
        distances = (-1,-1)
        r=rospy.Rate(10)
        time_start = rospy.get_time()
        time_now = rospy.get_time()
        
        print("Searching...")
        while (distances[0] == -1 and not rospy.is_shutdown()):    
            time_now=rospy.get_time()
            if (time_now-time_start > 10):
                # restart the current time after forward call & 10 seconds and 
                time_start=rospy.get_time() 
                print("No egg in sight, I'm outta here")
                # maintain 15 meter boundary
                if (self.position[1] + 0.5 < 15.0): 
                    halfMeter=(0.0,0.5)
                    self.go_to(halfMeter)
                else: 
                    # go back 10 meters and turn right 1 meter
                    # to check for new eggs in the location
                    wander=(1.0,-10.0)
                    self.go_to(wander)
            distances=ev.getDistance()
            r.sleep()
        return distances

    def shutdown(self):
        rospy.loginfo("Stopping the hunt.")

    def dummy(self, string):
        print string

if __name__ == '__main__':

    # TODO Implement the button usage
    global btn_state
    def ButtonEventCallback(self, data):
        if (data.state == ButtonEvent.RELEASED):
            btn_state = 0 #"released"
        else: 
            btn_state = 1 #"pressed"
        if (data.button  == ButtonEvent.Button0):
            print "Set state to 4 here."
            button = "B0"
        else:   
            button = "None"
        rospy.loginfo("Button %s was %s" % (button, state))
    
    # These need to be coded for each location you chose
    field_edge = (0.0, 0.5)
    basket = (0.0, 0.0)
    egg_location = (0.3,0.5)
    position = (0.0, 0.0)
    #rospy.init_node('Egg_Hunt', anonymous=False)
    
    hunter = EggHunter(field_edge, basket)
    serial = SerialTest()

    state = serial.read()
    print "Start. State is: " + state
    
    #use and compensate for serial connection and stack buffer
    while (state != None):
        if state is "44":
            state = "4"
        
        if state is "4":
            print "Scoop open. Sensing off."
            hunter.go_to_field()
            serial.change_state()
            egg_location = hunter.identify()
            print "Scoop Open. Enabling scoop sensing."
            ##state = serial.read()
            hunter.go_to(egg_location)

        if state is "1":
            print "Scoop open. Sensing Enabled."
            #hunter.go_to(egg_location)
            print "At egg location."
            ##state = serial.read()
            print("The current state: " + str(state))
            val = serial.read()
            
            if val != None:
                state = val
            else:
                hunter.go_to_edge_from_field()
                egg_location = hunter.identify()
                hunter.go_to(egg_location)
            #self.position = basket
###
# KEVIN: DO NOT CHANGE THIS FILE, make a copy...
###

        if state is "2":
            print "Closing. "
            state = serial.read()
            if not state == "3":
                state = "3"
                continue

        if state is "3":
            print "Scoop Closed. Ready to go home!"
            hunter.go_to_basket()
            print "At basket. Dropping egg."
            serial.change_state()
            ##state = serial.read()
            #hunter.set_state(state)
        state = serial.read()

    print("There is no state, yay time to restart!")
