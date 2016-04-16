#! /bin/bash/python

# Author:       Kevin Jin
# Contributer:  Jen Kniss
# Date:         March 2016
# Purpose:      For non-map based control of Kobuki base
import rospy
from geometry_msgs.msg import Twist


""" This the class that moves the Kobuki base according to a passed x and y 
    value. 
"""
class MoveIt():
    
    ANGULAR = 0.3925
    LINEAR = 0.0
    RATE = 10
    TIME_BASE = 8.5
    MIN_X = 0.0
    INCR_X = 0.1
    MIN_Z = 0.0
    Y_MULT = 10

    def __init__(self):
        rospy.init_node('rostime', anonymous=True)

    """ Move Kobuki forward
    """

    def forward(self, ydist):
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        move_cmd = Twist()
        r = rospy.Rate(self.RATE)
        direction=1
  
        if (ydist<0):
            self.turnRight()
            self.turnRight()
            ydist=abs(ydist)
            direction=-1
        
        move_cmd.linear.x = self.INCR_X
        ydist = ydist * self.Y_MULT*1.3 #1.18 is the error percentage
        
        if(ydist<0.5):
            ydist=ydist*1.5
        
        move_cmd.angular.z = self.MIN_Z
        start_time=rospy.get_time()
        time_now=rospy.get_time()
        
        while (time_now-start_time < ydist): 
            self.cmd_vel.publish(move_cmd)
            time_now=rospy.get_time()
            r.sleep()
        self.stop()     
	
        if(direction==-1):
            self.turnLeft()
            self.turnLeft()
            direction=1
    
    """ Go x-dist left/right in meters
    """
    def horizontal(self, xdist):
        
        if (xdist > self.MIN_X):
     	    self.turnRight()
            xdist=abs(xdist)
     	    self.forward(xdist)
     	    self.turnLeft()
     	elif (xdist < self.MIN_X):
     	    self.turnLeft()
            xdist = abs(xdist)
     	    self.forward(xdist)
            self.turnRight()
        else:
            print("MoveIt: No Horizontal Move Made")

    """ Turn kobuki Left
    """
    def turnLeft(self):
     	  cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     	  move_cmd = Twist()
     	  r = rospy.Rate(self.RATE)
     	  move_cmd.angular.z = self.ANGULAR
     	  move_cmd.linear.x = self.MIN_X
     	  start_time=rospy.get_time()
          time_now=rospy.get_time()

          while (time_now-start_time < self.TIME_BASE): 
              cmd_vel.publish(move_cmd)
              time_now=rospy.get_time()
              r.sleep()
          self.stop()
    
    """Turn Kobuki Right
    """
    def turnRight(self):
     	  cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     	  move_cmd = Twist()
     	  r = rospy.Rate(self.RATE)
     	  move_cmd.angular.z = -self.ANGULAR
     	  move_cmd.linear.x = self.MIN_X
     	  start_time=rospy.get_time()
          time_now=rospy.get_time()

          while (time_now-start_time < self.TIME_BASE): 
              cmd_vel.publish(move_cmd)
              time_now=rospy.get_time()
              r.sleep()
          self.stop()

    """Stop the Kobuki base
    """
    def stop(self):
        
        cmd_vel=rospy.Publisher('cmd_vel_mux/input/navi',Twist, queue_size=10)
        move_cmd=Twist()
        r=rospy.Rate(self.RATE)
        time_now=rospy.get_time()
        start_time=rospy.get_time()
        move_cmd.angular.z=0
        move_cmd.linear.x=self.MIN_X
        
        while(time_now-start_time<2):
            r.sleep()
            cmd_vel.publish(move_cmd)
            time_now=rospy.get_time()

