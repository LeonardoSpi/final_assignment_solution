#!/usr/bin/env python

import rospy
import random # This library is needed to computed random numbers
from random import randrange
# Import messages
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
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

velocity = target = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # The node can publish on topic
								     # /cmd_vel with a message
								     # of type Twist

def picktarget():		# This function is used to let the user pick a goal
				# among the possible targets defined above
	print("\n")
	while True:
		print("Please select one of the six possibile destinations:\n")
		print("1. (-4, -3)\n")
		print("2. (-4, -2)\n")
		print("3. (-4, 7)\n")
		print("4. (5, -7)\n")
		print("5. (5, -3)\n")
		print("6. (5, 1)\n")
		i = input("Digit a number between 1 and 6:  ")
		if i in [1, 2, 3, 4, 5 ,6]:	# If the user correctly pick a number between 1 and 6
			break			# go on, otherwise keep on spinning
		print("\n FORBIDDEN INPUT, TRY AGAIN \n")
	print("\n")
	print"You selected target: ", couples[i-1] # Print which target has been selected
	return i # Returns which couple number has been selected

def choose():		# This function is needed to let the user pick one of the possible
	print("\n")	# behaviors of the robot
	print("Please choose one robot behavior from the list:\n")
	print("1. Random target\n")
	print("2. Select target\n")
	print("3. Wall follower\n")
	print("4. Stand bye\n")
	choice = input("Digit a number between 1 and 4:  ")
	if choice not in [1,2,3,4]:	# If the user pick a number outside of [1,4] restart the function
		print("\n FORBIDDEN INPUT, TRY AGAIN  \n")
		choose()
	return choice # Return the selected behavior

def stop():			# This function is needed to stop the robot
	vel_msg = Twist()	# It publishes 0 velocities on the topic
	vel_msg.linear.x = 0	# cmd_vel with a message of type Twist
	vel_msg.linear.y = 0
	vel_msg.angular.z = 0
	velocity.publish(vel_msg)	

def behavior_server():
	rospy.init_node('user_interface') # Initialize the node as 'user_interface'

	# The node uses the services /wall_follower_switch, /random_goal_generator, and /picked_target

	srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
	srv_client_random_target_ = rospy.ServiceProxy('/random_goal_generator', SetBool)
	srv_client_picked_target_= rospy.ServiceProxy('/picked_target', behavior)

	rate = rospy.Rate(2)

	while not rospy.is_shutdown():
		choice = choose()	# Execute choose function to pick robot behavior

		if choice == 1:	# If user choose 1, random goal behavior is activated
			print("\nMoving toward the goal...")
			resp = srv_client_random_target_(True) # Calls for service /random_goal_generator
			if (resp.success) == True:
				print("\nGoal reached!")
				resp.success = False

		elif choice == 2: # If user choose 2, pick target behavior is activated
			i = picktarget()
			print("\nMoving toward the goal...")
			resp = srv_client_picked_target_(i) # Calls for service /picked_target
			if (resp.success) == True:
				print("\nGoal reached!")
				resp.success = False

		elif choice == 3: # If user choose 3, wall follower behavior is activated
			resp = srv_client_random_target_(False) # Calls for service /random_goal_generator
								# but this time with SetBool to False
								# so that the robot knows the previous goal
								# is no longer active

			resp = srv_client_wall_follower_(True)	# Calls for service /wall_follower_switch
			print("\n")
			print("Wall follower mode activated\n")	

			# At any time user can exit wall followe mode by pressing any key
	
			key = raw_input("Press any key to exit Wall follower mode: \n")
			if(len(key) >= 0):
				resp = srv_client_wall_follower_(False)	# Calls for services with
				resp = srv_client_random_target_(False) # SetBool to False
				stop() # Execute function to stop the robot
				continue

		elif choice == 4: # If user choose 4, stand bye mode is activated
			stop()# Execute function to stop the robot
			print("\n")
			print("Stand bye mode activated\n")	

			# At any time user can exit stand bye mode by pressing any key
	
			key = raw_input("Press any key to exit Stand bye mode: \n")
			if(len(key) >= 0):
				continue
		rate.sleep()
			

if __name__ == '__main__':
    try:
		behavior_server() # Execute function behavior_server()
    except rospy.ROSInterruptException: # Keep going until keyboard exception (Ctrl+C)
        	pass
