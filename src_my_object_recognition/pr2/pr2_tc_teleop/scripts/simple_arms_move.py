#!/usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes


# Point the head using controller

class Pr2MoveArm(object):

    def __init__(self):

        # Joint names
        self.right_arm_joint_names = ["right_shoulder_pan_joint",
                                    "right_shoulder_lift_joint",
                                    "right_upper_arm_roll_joint",
                                    "right_elbow_flex_joint",
                                    "right_forearm_roll_joint",
                                    "right_wrist_flex_joint",
                                    "right_wrist_roll_joint"]
        
        self.left_arm_joint_names = ["left_shoulder_pan_joint",
                                    "left_shoulder_lift_joint",
                                    "left_upper_arm_roll_joint",
                                    "left_elbow_flex_joint",
                                    "left_forearm_roll_joint",
                                    "left_wrist_flex_joint",
                                    "left_wrist_roll_joint"]


        self.right_gripper_joint_names = ["right_right_gripper_finger_joint",
                                        "right_left_gripper_finger_joint"]

        self.left_gripper_joint_names = ["left_right_gripper_finger_joint",
                                        "left_left_gripper_finger_joint"]


        self.l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]
        self.r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]
        
        self.l_arm_untucked = [ 0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
        self.r_arm_untucked = [-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
        
        self.l_arm_approach = [0.039, 1.1072, 0.0, -2.067, -1.231, -1.998, 0.369]
        self.r_arm_approach = [0.039, 1.1072, 0.0, -2.067, -1.231, -1.998, 0.369]


        self.gripper_open = [0.0,0.0]
        self.gripper_close = [0.05,0.05]
        

        self.right_arm_move_group = MoveGroupInterface("right_arm", "base_link")
        self.left_arm_move_group = MoveGroupInterface("left_arm", "base_link")

        self.right_gripper_move_group = MoveGroupInterface("right_gripper", "base_link")
        self.left_gripper_move_group = MoveGroupInterface("left_gripper", "base_link")


    def gripper(self, gripper_name, open):
        rospy.loginfo("Gripper...Open="+str(open))
        if gripper_name == "right_gripper":
            joints = self.right_gripper_joint_names
            if open:
                pose = self.gripper_open
            else:
                pose = self.gripper_close

            while not rospy.is_shutdown():
                result = self.right_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return
        else:
            joints = self.left_gripper_joint_names
            if open:
                pose = self.gripper_open
            else:
                pose = self.gripper_close

            while not rospy.is_shutdown():
                result = self.left_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return


    def approach(self, arm_name):
        rospy.loginfo("Aproach...")
        if arm_name == "right_arm":
            joints = self.right_arm_joint_names
            pose = self.r_arm_approach
            while not rospy.is_shutdown():
                result = self.right_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return
        else:
            joints = self.left_arm_joint_names
            pose = self.l_arm_approach
            while not rospy.is_shutdown():
                result = self.left_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return



    def tuck(self, arm_name):
        rospy.loginfo("Tuck...")

        if arm_name == "right_arm":
            joints = self.right_arm_joint_names
            pose = self.r_arm_tucked
            while not rospy.is_shutdown():
                result = self.right_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return
        else:
            joints = self.left_arm_joint_names
            pose = self.l_arm_tucked
            while not rospy.is_shutdown():
                result = self.left_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return

        

        
        rospy.loginfo("Tuck...Done")
    
    def untuck(self, arm_name):
        rospy.loginfo("Untuck...")
        if arm_name == "right_arm":
            joints = self.right_arm_joint_names
            pose = self.r_arm_untucked
            while not rospy.is_shutdown():
                result = self.right_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return
        else:
            joints = self.left_arm_joint_names
            pose = self.l_arm_untucked
            while not rospy.is_shutdown():
                result = self.left_arm_move_group.moveToJointPosition(joints, pose, 0.02)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return
        
        rospy.loginfo("Untuck...Done")

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    grasping_client = Pr2MoveArm()
    grasping_client.tuck("right_arm")
    grasping_client.tuck("left_arm")
    
    rospy.loginfo("Finished")
