#!/usr/bin/env python

import ros
import roslaunch
import rospy
import roslib
import tf
import argparse
import actionlib
import subprocess
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
        
        #print "Starting the position node"
        #self.roslauncher = roslaunch.ROSLaunch()
        #self.roslauncher.start()
        #self.pos_node = roslaunch.Node(package="get_position", node_type="get_position.py", name='position_')
        #self.roslauncher.launch(self.pos_node)
    
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
        #goal = MoveBaseGoal()
        #goal.target_pose.header.frame_id = 'map'
        #goal.target_pose.header.stamp = rospy.Time.now()
        #goal.target_pose.pose = Pose(Point(float(x), float(y), float(z)), Quaternion)
    
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
        #print "Position data: " + msg.pose.pose.position
        self.position = msg.pose.pose.position
        print "From Callback: " + str(self.position)
        return self.position 
        

    def get_my_position(self):
        #self.position = rospy.Subscriber('odom', Odometry, self.positionIs )
        position = rospy.Subscriber('odom', Odometry, self.positionIs)
        print "After call back: " + str(tuple(self.position))
        return self.position

    def get_egg_location(self):
        my_pose = self.get_my_position()
        print "My pose is: "
        print my_pose
        distances = self.identify()
        egg_location = self.translate(my_pose, distances)
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

