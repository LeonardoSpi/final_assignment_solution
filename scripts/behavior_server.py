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

def status(msg): # This function is the callback for the /gmapping odom subscriber
	global goal_status
	global active_
	desired_position_.x = rospy.get_param('des_pos_x') # Load the coordinates of the desired goal 
	desired_position_.y = rospy.get_param('des_pos_y') # from the parameter server
	desired_position_.z = 0
	distance_x = abs(desired_position_.x-msg.linear.x) # Compute the distances accessing
	distance_y = abs(desired_position_.y-msg.linear.y) # the current position from the msg passed by
							   # the callback

	if((distance_x<1.5) and (distance_y<1.5)): # If the distances in both axis if less than 1.5
		goal_status = True		   # the goal is acquired

	else: # If the distances are higher than 1.5
		if active_ == True: # Check if random target service is active
			goal_status = False 
			print "desired goal x: ", desired_position_.x # Random target service is active
			print "desired goal y: ", desired_position_.y # print all valuable
			print "current position x: ", msg.linear.x	# informations
			print "current position y: ", msg.linear.y
			print "Distance in x is: ", distance_x
			print "Distance in y is: ", distance_y

def randomtarget(req):	# This function gets called from the services /random_goal_generator
	global active_	
	active_ = req.data # Store the request on the global variable active_
	res = SetBoolResponse()
	position = rospy.Subscriber('gmapping_odom', Twist, status) # Subscribe to /gmapping_odom

	if active_ == True: # If the request of the service is set to True

		i = randrange(len(couples)) # Generate random coordinate couple
		print"Randomly selected target is: ", couples[i]
		x, y = couples[i] # Store coordinates in x and y variables
		move(x,y) # Call function move passing arguments x and y
		rate.sleep()
		rate.sleep()
		while not goal_status == True: # Wait until the goal has not been reached
			rate.sleep()	       
		res.success = True # The goal has been reached,
    		return res	   # return True

	elif active_ == False: # If the request of the service is set to False
		position.unregister() # unregister from topic /gmapping_odom
		goalid = GoalID() # goalid is of type GoalID
		cancel_pub.publish(goalid) # publish to move_base/cancel to delete previous goal
		res.success = False
		res.message = 'Mission aborted!'
		return res # Return False

def move(x,y): # This function is needed to give a new goal to move_base node

	move_base_goal = MoveBaseGoal() # move_base_goal is of type MoveBaseGoal()
	move_base_goal.target_pose.header.frame_id = 'map' # Set frame_id to map
	move_base_goal.target_pose.pose.position.x = x	   # Set x coordinate
	move_base_goal.target_pose.pose.position.y = y	   # Set y coordinate
	move_base_goal.target_pose.pose.position.z = 0.0   # Set z coordinate
	move_base_goal.target_pose.pose.orientation.w = 1  # Set w orientation
	goal = MoveBaseActionGoal() # goal is of type MoveBaseActionGoal
	goal.goal = move_base_goal  # put move_base_goal inside goal.goal
	target.publish(goal) # Publish goal
	rospy.set_param("des_pos_x", x) # Set des_pos_x parameter to x
        rospy.set_param("des_pos_y", y) # Set des_pos_y parameter to y

def pickedtarget(req): # This function gets called from the services /picked_target
	i = req.request - 1 # load the request into the variable i
	x, y = couples[i] # Extract x and y coordinates from couple number i define above
	move(x, y) # Execute move function passing arguments x and y
	rate.sleep()
	rate.sleep()
	while not goal_status == True: # Wait until the goal is reached
		rate.sleep()
	success = True # The goal is reached,
    	return success # return True

def goal_server():
		
	s = rospy.Service('/random_goal_generator', SetBool, randomtarget) # The node works as a server for the
								 	   # service /random_goal_generator
									   # with callback randomtarget

	s = rospy.Service('/picked_target', behavior, pickedtarget) # The node works as a server for the
								    # service /picked_target
								    # with callback pickedtarget

	print("Ready to generate goal:") # Once the node is started, print to screen so that
					 # the user knows it is ready to generate a goal

	rospy.spin() # Continue to cycle


if __name__ == '__main__':
    try:
        goal_server() # Execute function goal_server()
    except rospy.ROSInterruptException: # Keep going until keyboard exception (Ctrl+C)
        pass
