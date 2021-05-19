#!/usr/bin/env python
# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/

import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Goal:
    def __init__(self, frame_id, distance = 0.5):
        self.frame_id = frame_id
        self.distance = distance
        self.x_old = 0
        self.y_old = 0

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('/summit_xl/move_base',MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        rospy.loginfo("Goal class initialized")


    def movebase_client(self, poseStamped):
        # Creates a new goal with the MoveBaseGoal constructor    
        goal = MoveBaseGoal()
        goal.target_pose = poseStamped
        # goal.target_pose.header.frame_id = poseStamped.header.frame_id
        # goal.target_pose.header.stamp = rospy.Time.now()
    

        # # # Calculate the correct position x
        # multiply_x = 1
        # if pose.position.x > self.x_old:
        #     multiply_x = -1
        # # Calculate the correct position y
        # multiply_y = 1
        # if pose.position.y > self.y_old:
        #     multiply_y = -1

        # # Move {{distance}} meters forward along the x axis of the "map" coordinate frame 
        # goal.target_pose.pose.position.x = pose.position.x + self.distance * multiply_x
        # goal.target_pose.pose.position.y = pose.position.y + self.distance * multiply_y

        # # No rotation of the mobile base frame w.r.t. map frame
        # # goal.target_pose.pose.orientation.w = 0.302
        # goal.target_pose.pose.orientation.w = pose.orientation.w
        
        # # Update old x and y
        # self.x_old = pose.position.x
        # self.y_old = pose.position.y
        rospy.loginfo("Move goal: %s" % goal)

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return self.client.get_result()   
