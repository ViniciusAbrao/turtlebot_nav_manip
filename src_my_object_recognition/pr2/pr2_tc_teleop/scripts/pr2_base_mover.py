#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64


class Pr2BaseMover(object):

    def __init__(self):
        
        rospy.loginfo("PR2 JointMover Initialising...")

        # Orientation
        self.pub_bl_caster_rotation_joint_position = rospy.Publisher(
            '/pr2/bl_caster_rotation_joint_controller/command',
            Float64,
            queue_size=1)
        self.pub_br_caster_rotation_joint_position = rospy.Publisher(
            '/pr2/br_caster_rotation_joint_controller/command',
            Float64,
            queue_size=1)
        self.pub_fl_caster_rotation_joint_position = rospy.Publisher(
            '/pr2/fl_caster_rotation_joint_controller/command',
            Float64,
            queue_size=1)
        self.pub_fr_caster_rotation_joint_position = rospy.Publisher(
            '/pr2/fr_caster_rotation_joint_controller/command',
            Float64,
            queue_size=1)

        self.direction_array = [self.pub_bl_caster_rotation_joint_position,
                                self.pub_br_caster_rotation_joint_position,
                                self.pub_fl_caster_rotation_joint_position,
                                self.pub_fr_caster_rotation_joint_position]

        # Speed
        self.pub_bl_caster_l_wheel_joint_velocity = rospy.Publisher(
            '/pr2/bl_caster_l_wheel_joint_controller/command',
            Float64,
            queue_size=1)
        self.pub_bl_caster_r_wheel_joint_velocity = rospy.Publisher(
            '/pr2/bl_caster_r_wheel_joint_controller/command',
            Float64,
            queue_size=1)        
        self.pub_br_caster_l_wheel_joint_velocity = rospy.Publisher(
            '/pr2/br_caster_l_wheel_joint_controller/command',
            Float64,
            queue_size=1)
        self.pub_br_caster_r_wheel_joint_velocity = rospy.Publisher(
            '/pr2/br_caster_r_wheel_joint_controller/command',
            Float64,
            queue_size=1)        

        self.pub_fl_caster_l_wheel_joint_velocity = rospy.Publisher(
            '/pr2/fl_caster_l_wheel_joint_controller/command',
            Float64,
            queue_size=1)
        self.pub_fl_caster_r_wheel_joint_velocity = rospy.Publisher(
            '/pr2/fl_caster_r_wheel_joint_controller/command',
            Float64,
            queue_size=1)

        self.pub_fr_caster_l_wheel_joint_velocity = rospy.Publisher(
            '/pr2/fr_caster_l_wheel_joint_controller/command',
            Float64,
            queue_size=1)
        self.pub_fr_caster_r_wheel_joint_velocity = rospy.Publisher(
            '/pr2/fr_caster_r_wheel_joint_controller/command',
            Float64,
            queue_size=1)

        self.speed_array = [self.pub_bl_caster_l_wheel_joint_velocity,
                            self.pub_bl_caster_r_wheel_joint_velocity,
                            self.pub_br_caster_l_wheel_joint_velocity,
                            self.pub_br_caster_r_wheel_joint_velocity,
                            self.pub_fl_caster_l_wheel_joint_velocity,
                            self.pub_fl_caster_r_wheel_joint_velocity,
                            self.pub_fr_caster_l_wheel_joint_velocity,
                            self.pub_fr_caster_r_wheel_joint_velocity]

        self.pub_torso_lift_joint_position = rospy.Publisher(
            '/pr2/torso_lift_joint_controller/command',
            Float64,
            queue_size=1)

        #self._check_all_publishers()

    def _check_all_publishers(self):
        """
        Check all publishers
        """
        rospy.logdebug("Checking Direction Publishers...")
        for publisher_obj in self.direction_array:
            self._check_publisher_connection(publisher_obj)
        rospy.logdebug("Checking Direction Publishers...DONE")

        rospy.logdebug("Checking Speed Publishers...")
        for publisher_obj in self.speed_array:
            self._check_publisher_connection(publisher_obj)
        rospy.logdebug("Checking Speed Publishers...DONE")

        rospy.logdebug("Checking Torso Publisher...")
        self._check_publisher_connection(self.pub_torso_lift_joint_position)
        rospy.logdebug("Checking Torso Publisher...DONE")

        rospy.logdebug("All Publishers READY")
            
    def _check_publisher_connection(self, publisher_object):
        """
        Checks that the publisher is working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while publisher_object.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("Publisher Connected")

        
    def move_torso(self, torso_height):
        torso_msg = Float64()
        torso_msg.data = torso_height
        self.pub_torso_lift_joint_position.publish(torso_msg)

    def move(self, direction_angle, lin_speed, pivot=False, stop=False):

        rospy.logdebug("direction_angle="+str(direction_angle))
        rospy.logdebug("lin_speed="+str(lin_speed))
        
        speed_msg = Float64()
        direction_msg = Float64()        
        
        if stop:
            for speed_pub in self.speed_array:
                speed_pub.publish(speed_msg)
        else:
            speed_msg.data = lin_speed
            direction_msg.data = direction_angle
            if pivot:
                direction_msg.data = 0.7
                direction_inverse_msg = Float64()
                direction_inverse_msg.data = -0.7

                speed_inverse_msg = Float64()
                speed_inverse_msg.data = -1.0 * lin_speed

                self.pub_bl_caster_rotation_joint_position.publish(direction_msg)
                self.pub_br_caster_rotation_joint_position.publish(direction_inverse_msg)
                self.pub_fl_caster_rotation_joint_position.publish(direction_inverse_msg)
                self.pub_fr_caster_rotation_joint_position.publish(direction_msg)

                self.pub_bl_caster_l_wheel_joint_velocity.publish(speed_inverse_msg)
                self.pub_bl_caster_r_wheel_joint_velocity.publish(speed_inverse_msg)
                self.pub_br_caster_l_wheel_joint_velocity.publish(speed_msg)
                self.pub_br_caster_r_wheel_joint_velocity.publish(speed_msg)
                self.pub_fl_caster_l_wheel_joint_velocity.publish(speed_inverse_msg)
                self.pub_fl_caster_r_wheel_joint_velocity.publish(speed_inverse_msg)
                self.pub_fr_caster_l_wheel_joint_velocity.publish(speed_msg)
                self.pub_fr_caster_r_wheel_joint_velocity.publish(speed_msg)


            else:
                for dir_pub in self.direction_array:
                    dir_pub.publish(direction_msg)
                
                for speed_pub in self.speed_array:
                    speed_pub.publish(speed_msg)


def test1():
    rospy.init_node('pr2_mover_demo', anonymous=True)
    pr2_mover = Pr2BaseMover()
    pr2_mover.move(direction_angle=0.0, lin_speed=10.0)
    time.sleep(5.0)
    pr2_mover.move(direction_angle=0.0, lin_speed=0.0)
    time.sleep(1.0)
    pr2_mover.move(direction_angle=1.92, lin_speed=10.0)
    time.sleep(5.0)
    pr2_mover.move(direction_angle=0.0, lin_speed=0.0)
    time.sleep(1.0)
    pr2_mover.move(direction_angle=1.92, lin_speed=-10.0)
    time.sleep(5.0)
    pr2_mover.move(direction_angle=0.0, lin_speed=0.0)
    time.sleep(1.0)

def test2():
    rospy.init_node('pr2_mover_demo', anonymous=True)
    pr2_mover = Pr2BaseMover()

    while not rospy.is_shutdown():
        pr2_mover.move(direction_angle=0.0, lin_speed=100.0, pivot=True)
        time.sleep(5.0)
        pr2_mover.move(direction_angle=0.0, lin_speed=-100.0, pivot=True)
        time.sleep(5.0)


if __name__ == "__main__":
    #test1()
    test2()

