#!/usr/bin/env python

import rospy
import random # This library is needed to computed random numbers
from random import randrange
# Import messages
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
# Import services and behavior custom service
from std_srvs.srv import *
from final_assignment_solution.srv import behavior, behaviorResponse

# Define coordinates of possibile targets
coordinate1 = (-4, -3)
coordinate2 = (-4, -2)
coordinate3 = (-4, 7)
coordinate4 = (5, -7)
coordinate5 = (5, -3)
coordinate6 = (5, 1)
couples = [coordinate1, coordinate2, coordinate3, coordinate4, coordinate5, coordinate6]

desired_position_ = Point() # desired_position_ is of type Point()

target = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1) # The node can publish on topic
									     # move_base/goal with a message
									     # of type MoveBaseActionGoal

cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10) # The node can publish on topic
									# move_base/cancel with a message
									# of type GoalID

rospy.init_node('behavior_server') # Initialize the node as behavior_server

rate = rospy.Rate(6)

goal_status = False

active_ = False

def random_number_generator(): # This function returns a random number between 1 and 6
	n = random.uniform(1,6)
	return n

def status(msg):
	global goal_status
	global active_
	desired_position_.x = rospy.get_param('des_pos_x')
	desired_position_.y = rospy.get_param('des_pos_y')
	desired_position_.z = 0
	distance_x = abs(desired_position_.x-msg.linear.x)
	distance_y = abs(desired_position_.y-msg.linear.y)
	if((distance_x<1.5) and (distance_y<1.5)):
		goal_status = True

	else:
		if active_ == True:
			goal_status = False
			print ("desired goal x: ", desired_position_.x)
			print ("desired goal y: ", desired_position_.y)
			print ("current position x: ", msg.linear.x)
			print ("current position y: ", msg.linear.y)
			print ("Distance in x is: ", distance_x)
			print ("Distance in y is: ", distance_y)

def randomtarget(req):
	global active_
	active_ = req.data
	res = SetBoolResponse()
	position = rospy.Subscriber('gmapping_odom', Twist, status)
	print req.data

	if active_ == True:
		print("\n")
		print"registered"
		i = randrange(len(couples))
		print"Randomly selected target is: ", couples[i]
		x, y = couples[i]
		move(x,y)
		rate.sleep()
		rate.sleep()
		while not goal_status == True:
			rate.sleep()
		res.success = True
    		return res

	elif active_ == False:
		position.unregister()
		goalid = GoalID()
		cancel_pub.publish(goalid)
		print"unregistered"
		res.success = False
		res.message = 'Mission aborted!'
		return res

def move(x,y):
	move_base_goal = MoveBaseGoal()
	move_base_goal.target_pose.header.frame_id = 'map'
	move_base_goal.target_pose.pose.position.x = x
	move_base_goal.target_pose.pose.position.y = y
	move_base_goal.target_pose.pose.position.z = 0.0
	move_base_goal.target_pose.pose.orientation.w = 1
	goal = MoveBaseActionGoal()
	goal.goal = move_base_goal
	target.publish(goal)
	rospy.set_param("des_pos_x", x)
        rospy.set_param("des_pos_y", y)

def pickedtarget(req):
	i = req.request - 1
	x, y = couples[i]
	move(x, y)
	rate.sleep()
	rate.sleep()
	while not goal_status == True:
		rate.sleep()
	success = True
    	return success

def goal_server():

	rospy.init_node('goal_server') # Initialize the node as goal_server
		
	s = rospy.Service('/random_goal_generator', SetBool, randomtarget) # The nodes works as a server for the
								 	   # service /random_goal_generator

	s = rospy.Service('/picked_target', behavior, pickedtarget) # The nodes works as a server for the
								    # service /picked_target

	print("Ready to generate goal:") # Once the node is started, print to screen so that
					 # the user knows it is ready to generate a goal

	rospy.spin() # Continue to cycle


if __name__ == '__main__':
    try:
        goal_server() # Execute function goal_server()
    except rospy.ROSInterruptException: # Keep going until keyboard exception (Ctrl+C)
        pass
