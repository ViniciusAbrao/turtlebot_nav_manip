#! /usr/bin/env python

import rospy
import time
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
import moveit_commander

class Move_robot():

    def __init__(self,x,y,w,z):
    
        # create the connection to the action server
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # waits until the action server is up and running
        self.client.wait_for_server()

        # creates a goal to send to the action server
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w
        goal.target_pose.pose.orientation.z = z

        # sends the goal to the action server, specifying which feedback function
        # to call when feedback received
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        rospy.on_shutdown(self.stop_action_client)

        self.client.wait_for_result()
        rospy.loginfo('[Result] State: %d'%(self.client.get_state()))

    def feedback_callback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))
        
    def stop_action_client(self):
        self.client.cancel_goal()
        
class ArmSimpleTrajectory():

    def __init__(self,j1,j2,j3,j4):
        
        group = moveit_commander.MoveGroupCommander("arm")
        group_variable_values = group.get_current_joint_values()

        group_variable_values[0] = j1
        group_variable_values[1] = j2
        group_variable_values[2] = j3
        group_variable_values[3] = j4

        group.set_joint_value_target(group_variable_values)

        group.go(wait=True)
      
        rospy.sleep(5)

class GripperTrajectory():

    def __init__(self,value):
        
        group = moveit_commander.MoveGroupCommander("gripper")
        group_variable_values = group.get_current_joint_values()
        group_variable_values= [value, value]

        group.set_joint_value_target(group_variable_values)
        group.go(wait=True)

        rospy.sleep(5)

if __name__ == '__main__':

    rospy.init_node('full_node', anonymous=True)
    rospy.Rate(1)
      
    Move_robot(0.675348545334,-0.231368268314,0.0759178705799,-0.997114074179)
    rospy.loginfo("Move_robot done")

    GripperTrajectory(0.015)
    rospy.loginfo("Gripper opened")

    ArmSimpleTrajectory(0.0, 0.55, 0.5, -1.1)
    rospy.loginfo("Target Position")

    GripperTrajectory(0.005)
    rospy.loginfo("Gripper closed")

    ArmSimpleTrajectory(0.0, -1.0, 0.3, 0.7)
    rospy.loginfo("Home Position")

    Move_robot(3.7,-4.47,0.99941,-0.03436)
    rospy.loginfo("Move_robot done")

    ArmSimpleTrajectory(0.0, 0.55, 0.5, -1.1)
    rospy.loginfo("Target Position")

    GripperTrajectory(0.015)
    rospy.loginfo("Gripper opened")

    ArmSimpleTrajectory(0.0, -1.0, 0.3, 0.7)
    rospy.loginfo("Home Position")

    GripperTrajectory(0.005)
    rospy.loginfo("Gripper closed")

    moveit_commander.roscpp_shutdown()
