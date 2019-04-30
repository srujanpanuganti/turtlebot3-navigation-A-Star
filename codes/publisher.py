#!/usr/bin/env python
# above command need to be mentioned in every python scripts when using with ROS

#importing rospy to use ROS with python client
import rospy
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
# We import the Twist topic from the geometry_msgs to publish our messages to the turtlebot
from geometry_msgs.msg import Twist
import numpy as np


res = 15
# ROBOT_INITIAL_POSE="-x 5 -y 2" roslaunch planning sru.launch

list_of = []

with open('/home/srujan/catkin_ws/src/planning/scripts/converted_velocities.txt') as f:
    for line in f:
        inner_list = [elt.strip() for elt in line.split('\t')]
        list_of.append(inner_list)


list_arr = np.asarray(list_of).astype(float)

#we create a node called listener
rospy.init_node('listener',anonymous = True)

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

move = Twist()
rate = rospy.Rate(1)
current_pos = Pose()

def mover():
	count = 0
	for vel in list_arr:

		move.linear.x = vel[0]/res
		move.linear.y = 0
		move.linear.z =  0
		move.angular.x = 0
		move.angular.y = 0
		move.angular.z = vel[5]*0.75
		count +=1		
		print("count", count)

		print('publishing')

		start_time = rospy.Time.now()
		duration = rospy.Duration(1)			#0.5
		end_time = start_time + duration
		iter1 = 0
		while rospy.Time.now() < end_time:
			iter1 +=1
			pub.publish(move)
			rate.sleep()


mover()
rospy.spin()

