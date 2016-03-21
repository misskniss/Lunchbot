#!/usr/bin/env python

import ros
import roslaunch
import rospy
import roslib
import tf
import argparse
import actionlib
import subprocess
import time
import egg_vision
import get_position
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sys import executable


class EggHunter():

    def __init__(self, field, basket):

        self.DETECT = 1
        self.DROP = 0
        self.FIELD_EDGE = field
        self.BASKET = basket
        self.position = (-20,-20,0)
        print self.FIELD_EDGE
        print self.BASKET

        
        rospy.init_node('Egg_Hunt', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("waiting 5 sec for the action server to come up...")
        self.move_base.wait_for_server(rospy.Duration(5))
        
    
    def go_to_field(self):
        self.go_to(self.FIELD_EDGE)
        print "At field."

    def hunt(self):
        self.go_to_field()
        self.detect_egg()
        self.fetch_egg()
        self.go_home()
        self.drop_egg()

    def detect_egg(self):
        self.set_scooper_state(self.DETECT)
        print "Scoop is detecting eggs."
    
    def fetch_egg(self):
        self.go_to(self.get_egg_location())
        print "Fetching Egg..."
    
    def go_home(self):
        self.go_to(self.BASKET)
        print "At Basket."
    
    def drop_egg(self):
        self.set_scooper_state(self.DROP)
        print "Egg in basket."
    
   
    def go_to(self, target):
        
        print "Moving to " + str(tuple(target))
        x, y, z = target
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(float(x), float(y), float(z)), Quaternion(0.000,0.000,0.892,-1.500))
        
        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(240))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("Failed to reach the goal. Target cancelled.")
        else: 
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached goal!")
    
    def identify(self):
        #distances = EggVision.search()
        print "Identifying... "
        return (-3, 2)  # Represents egg is 3ft to the left and 2 feet forward

    def translate(self, current_position, vision_results):
        #new_coordinates = my current position + distances for x and y

        print "Translating distances" + str(tuple(vision_results)) + " with current pose " + str(tuple(current_position))
        fake_new_coordinates =(-2.19, 1.98, 0)
        return fake_new_coordinates
    
    def set_scooper_state(self, state):
        # send a signal to the scoop to indicate travelling or ready to hunt
        if (state is self.DROP):
            print "Setting Scoop state to DROP"
        elif (state is self.DETECT):
            print "Setting Scoop to DETECT"
        else:
            print "Unknown scoop state. Ignoring request."

    def positionIs(self, msg):
        self.position = msg.pose.pose.position
        sub_obj.unregister()
        #print "### Inside Callback: " + str(self.position)
        return self.position
        

    def get_my_position(self):
        global sub_obj
        sub_obj = rospy.Subscriber('odom', Odometry, self.positionIs)
        ##self.position = rospy.wait_for_message('odom/pose/pose/position/x', Odometry, 10)
        time.sleep(5)
        #print "after calback: " + str(self.position)
        return self.position

    def get_egg_location(self):
        my_pose = self.get_my_position()
        print "My pose is: "
        my_point = (my_pose.x, my_pose.y, my_pose.z)
        print my_pose.x, my_pose.y, my_pose.z
        distances = self.identify()
        egg_location = self.translate(my_point, distances)
        print "Found egg location."
        return egg_location

    def shutdown(self):
        rospy.loginfo("Stopping the hunt.")

    def dummy(self, string):
        print string

if __name__ == '__main__':
    
    # These need to be coded for each map you make
    field_edge = (-1.36, 0.839, 0)
    basket = (-0.371, 0.059, 0)
    
    hunter = EggHunter(field_edge, basket)
    hunter.hunt()

