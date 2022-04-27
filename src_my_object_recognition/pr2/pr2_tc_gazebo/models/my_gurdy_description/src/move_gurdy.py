#!/usr/bin/env python

import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/gurdy/head_upperlegM1_joint_position_controller/command
/gurdy/head_upperlegM2_joint_position_controller/command
/gurdy/head_upperlegM3_joint_position_controller/command
/gurdy/upperlegM1_lowerlegM1_joint_position_controller/command
/gurdy/upperlegM2_lowerlegM2_joint_position_controller/command
/gurdy/upperlegM3_lowerlegM3_joint_position_controller/command
"""

class gurdyJointMover(object):

    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("Gurdy JointMover Initialising...")

        self.pub_upperlegM1_joint_position = rospy.Publisher(
            '/gurdy/head_upperlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_upperlegM2_joint_position = rospy.Publisher(
            '/gurdy/head_upperlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_upperlegM3_joint_position = rospy.Publisher(
            '/gurdy/head_upperlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_lowerlegM1_joint_position = rospy.Publisher(
            '/gurdy/upperlegM1_lowerlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_lowerlegM2_joint_position = rospy.Publisher(
            '/gurdy/upperlegM2_lowerlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_lowerlegM3_joint_position = rospy.Publisher(
            '/gurdy/upperlegM3_lowerlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        joint_states_topic_name = "/gurdy/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.gurdy_joints_callback)
        gurdy_joints_data = None
        rate = rospy.Rate(2)
        while gurdy_joints_data is None:
            try:
                gurdy_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass
            rate.sleep()

        self.gurdy_joint_dictionary = dict(zip(gurdy_joints_data.name, gurdy_joints_data.position))

    def move_gurdy_all_joints(self, upperlegM1_angle, upperlegM2_angle, upperlegM3_angle, lowerlegM1_value, lowerlegM2_value ,lowerlegM3_value):
        upperlegM1 = Float64()
        upperlegM1.data = upperlegM1_angle
        upperlegM2 = Float64()
        upperlegM2.data = upperlegM2_angle
        upperlegM3 = Float64()
        upperlegM3.data = upperlegM3_angle

        lowerlegM1 = Float64()
        lowerlegM1.data = lowerlegM1_value
        lowerlegM2 = Float64()
        lowerlegM2.data = lowerlegM2_value
        lowerlegM3 = Float64()
        lowerlegM3.data = lowerlegM3_value

        self.pub_upperlegM1_joint_position.publish(upperlegM1)
        self.pub_upperlegM2_joint_position.publish(upperlegM2)
        self.pub_upperlegM3_joint_position.publish(upperlegM3)

        self.pub_lowerlegM1_joint_position.publish(lowerlegM1)
        self.pub_lowerlegM2_joint_position.publish(lowerlegM2)
        self.pub_lowerlegM3_joint_position.publish(lowerlegM3)


    def gurdy_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.gurdy_joint_dictionary = dict(zip(msg.name, msg.position))


    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def gurdy_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'base_waist_joint', 'body_head_joint', 'waist_body_joint is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param joint_name:
        :param value:
        :param error: In radians
        :return:
        """
        joint_reading = self.gurdy_joint_dictionary.get(joint_name)
        if not joint_reading:
            print "self.gurdy_joint_dictionary="+str(self.gurdy_joint_dictionary)
            print "joint_name===>"+str(joint_name)
            assert "There is no data about that joint"
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar

    def gurdy_movement_look(self, upperlegM1_angle, upperlegM2_angle, upperlegM3_angle, lowerlegM1_value, lowerlegM2_value ,lowerlegM3_value):
        """
        Move:
        'head_upperlegM1_joint',
        'head_upperlegM2_joint',
        'head_upperlegM3_joint',
        'upperlegM1_lowerlegM1_joint',
        'upperlegM2_lowerlegM2_joint',
        'upperlegM3_lowerlegM3_joint'
        :return:
        """
        check_rate = 5.0
        position_upperlegM1 = upperlegM1_angle
        position_upperlegM2 = upperlegM2_angle
        position_upperlegM3 = upperlegM3_angle

        position_lowerlegM1 = lowerlegM1_value
        position_lowerlegM2 = lowerlegM2_value
        position_lowerlegM3 = lowerlegM3_value

        similar_upperlegM1 = False
        similar_upperlegM2 = False
        similar_upperlegM3 = False

        similar_lowerlegM1 = False
        similar_lowerlegM2 = False
        similar_lowerlegM3 = False

        rate = rospy.Rate(check_rate)
        while not (similar_upperlegM1 and similar_upperlegM2 and similar_upperlegM3 and similar_lowerlegM1 and similar_lowerlegM2 and similar_lowerlegM3):
            self.move_gurdy_all_joints(position_upperlegM1,
                                       position_upperlegM2,
                                       position_upperlegM3,
                                       position_lowerlegM1,
                                       position_lowerlegM2,
                                       position_lowerlegM3)
            similar_upperlegM1 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM1_joint",
                                                                         value=position_upperlegM1)
            similar_upperlegM2 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM2_joint",
                                                                         value=position_upperlegM2)
            similar_upperlegM3 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM3_joint",
                                                                         value=position_upperlegM3)
            similar_lowerlegM1 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM1_lowerlegM1_joint",
                                                                         value=position_lowerlegM1)
            similar_lowerlegM2 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM2_lowerlegM2_joint",
                                                                         value=position_lowerlegM2)
            similar_lowerlegM3 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM3_lowerlegM3_joint",
                                                                         value=position_lowerlegM3)

            rate.sleep()

    def gurdy_init_pos_sequence(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upperlegM1_angle = -1.55
        upperlegM2_angle = -1.55
        upperlegM3_angle = -1.55
        lowerlegM1_angle = 0.0
        lowerlegM2_angle = 0.0
        lowerlegM3_angle = 0.0
        self.gurdy_movement_look(upperlegM1_angle,
                                 upperlegM2_angle,
                                 upperlegM3_angle,
                                 lowerlegM1_angle,
                                 lowerlegM2_angle,
                                 lowerlegM3_angle)

        lowerlegM1_angle = -1.55
        lowerlegM2_angle = -1.55
        lowerlegM3_angle = -1.55
        self.gurdy_movement_look(upperlegM1_angle,
                                 upperlegM2_angle,
                                 upperlegM3_angle,
                                 lowerlegM1_angle,
                                 lowerlegM2_angle,
                                 lowerlegM3_angle)

    def gurdy_hop(self, num_hops=15):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """

        upper_delta = 1
        basic_angle = -1.55
        angle_change = random.uniform(0.2, 0.7)
        upperlegM_angle = basic_angle
        lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0

        #self.gurdy_init_pos_sequence()
        for repetitions in range(num_hops):
            self.gurdy_movement_look(upperlegM_angle,
                                     upperlegM_angle,
                                     upperlegM_angle,
                                     lowerlegM_angle,
                                     lowerlegM_angle,
                                     lowerlegM_angle)

            upper_delta *= -1
            if upper_delta < 0:
                upperlegM_angle = basic_angle + angle_change
            else:
                upperlegM_angle = basic_angle
            lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0


    def gurdy_moverandomly(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upperlegM1_angle = random.uniform(-1.55, 0.0)
        upperlegM2_angle = random.uniform(-1.55, 0.0)
        upperlegM3_angle = random.uniform(-1.55, 0.0)
        lowerlegM1_angle = random.uniform(-2.9, pi/2)
        lowerlegM2_angle = random.uniform(-2.9, pi/2)
        lowerlegM3_angle = random.uniform(-2.9, pi/2)
        self.gurdy_movement_look(upperlegM1_angle,
                                 upperlegM2_angle,
                                 upperlegM3_angle,
                                 lowerlegM1_angle,
                                 lowerlegM2_angle,
                                 lowerlegM3_angle)

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving Gurdy...")
        while not rospy.is_shutdown():
            self.gurdy_init_pos_sequence()
            #self.gurdy_moverandomly()
            self.gurdy_hop()

if __name__ == "__main__":
    gurdy_jointmover_object = gurdyJointMover()
    gurdy_jointmover_object.movement_random_loop()

