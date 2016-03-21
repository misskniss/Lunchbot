#!/usr/bin/env python


#TurtleBot must have minimal.launch & amcl_demo.launch running prior to starting this script.
#http://answers.ros.org/question/79851/python-odometry/

import rospy
import roslib
import tf
import argparse
import actionlib
import subprocess
import egg_vision
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sys import executable
    

class EggHunter():
    def __init__(self, target):
        # note: target is coming in as a list and the Points() needs each item as a float.
        x, y, z = target
        rospy.init_node('Jen_GoTo', anonymous=False)

	#what to do if shut down (e.g. ctrl + C or failure)
	rospy.on_shutdown(self.shutdown)

	
	#tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("wait for the action server to come up")
	#allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

	#we'll send a goal to the robot to tell it to move to a
        #pose that's near the docking station
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	#customize the following Point() values so they are appropriate for your location
	goal.target_pose.pose = Pose(Point(float(x), float(y), float(z)), Quaternion(0.000, 0.000, 0.892, -1.500))

	#start moving
        self.move_base.send_goal(goal)

	#allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(240)) 


	if not success:
                self.move_base.cancel_goal()
                rospy.loginfo("The base failed to reach the desired pose")
    	else:
		# We made it!
		state = self.move_base.get_state()
		if state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Hooray, reached the desired pose")

    """ Go to a coordinate. This can be a coordinate from the translate(), HOME, or FIELD
    """
    def go_to():

    """ Call the function the Vision Team have proided called EggVision.search(). Search returns a tuple of 
        distance to travel along the X axis and distance to travel on the Y axis. 
        NOT to be confused with an actual point. 
        For instance, if EggVision.search() returns the tuple (-3,2) this means move left 3 and forward 2.
        That value is turned into a point to navigate to with the translate function.
    """
    def identify():
        distances = EggVision.search()

    """ Takes the tuple representing how far to move left/right and forward and translates it to a 
        real coordinate based on where we currently are.
    """
    def translate():

    """ Sets the scoops state to either DETECT or DROP. These are signals TO the scoop mechanism.
        DETECT  = 1 and represents the scoop open with the sonic sensor enabled 
        DROP    = 0 and represents the scoop in a closed state moving to an open state with the sonic sensor
                    disabled
        Note: the states where the sensor has detected an egg and when the scoop closes as a result are 
              signaled as separate incoming signals FROM the scoop mechanism. 
    """
    def set_scoop_state();

    """ Gets the current state of the scoop.
        State 1: open and DETECT
        State 2: Found and egg and closing the scoop, sonic sensor diabled
        State 3: Closed and holding an egg, sonic sensor disabled
        State 4: DROP (open) and sonic sensor disabled
    """
    def get_scoop_state():


    def shutdown(self):
        rospy.loginfo("Stop")

    def dummy(self, string):
        print string



if __name__ == '__main__':

    
    parser = argparse.ArgumentParser()
    parser.add_argument("room", help="example: MEC202P")
    args = parser.parse_args()
    
    def fetch_location_specs(location):
        print "Looking up: " + location
        try: 
            locations = open('./LOCATIONS.txt', 'r')
            lines = iter(locations)
            for line in lines:
                if line.startswith(location):
                    print "Found match for " + location 
                    assoc_map = next(lines)
                    dock_points = next(lines).rstrip('\n')
                    room_point = next(lines).rstrip('\n')
                    #print "map is: " + assoc_map + "At Coordinates: " + room_point + " Dock Location: " + dock_points
                    return assoc_map,room_point,dock_points
        except rospy.ROSInterruptException: 
            rospy.loginfo("Exception thrown")

    
    location_specs = fetch_location_specs(args.room)
    the_map,room,dock = location_specs
    print "Map: " + the_map + "Target: " + room + "Home: " + dock
    print "Loading map..."
    map_call = "loadmap " + the_map
    print "Heading to location..."
    target = room.split(",")
    print target 
    GoTo(target)
    print "Returning Home..."
    near_dock = dock.split(",")
    GoTo(near_dock)
    #subprocess.Popen('charger', shell=True, executable='/usr/local/bin/interactive_bash')
    subprocess.Popen('chargetime', shell=True, executable='/usr/local/bin/interactive_bash')


