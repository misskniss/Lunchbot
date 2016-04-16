#!/usr/bin/env python
import ros
import roslib
import rospy

from nav_msgs.msg import Odometry

def odometryCb(msg):
    print msg.pose.pose.position

def get_position():
    rospy.init_node('odometry', anonymous=True)
    position = rospy.Subscriber('odom', Odometry, odometryCb)
    ros.spinOnce()
    return position

#if __name__ == "__main__":
#    rospy.init_node('odometry', anonymous=True)
#    position = rospy.Subscriber('odom', Odometry, odometryCb)
#    #rospy.spin()
#    ros.spinOnce()
#    #print position
