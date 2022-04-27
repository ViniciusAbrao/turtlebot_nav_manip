#!/usr/bin/env python
import time
import rospy
import time
from geometry_msgs.msg import Twist
from pr2_base_mover import Pr2BaseMover
import math

class Pr2TeleopBridge(object):

    def __init__(self):
        
        rospy.loginfo("PR2 Teleop Bridge Initialising...")
        self.pr2_mover = Pr2BaseMover()
        self.pr2_cmd_vel_topic = "/pr2/cmd_vel"

        rospy.Subscriber(self.pr2_cmd_vel_topic, Twist, self._cmd_vel_callback)

    def _check_cmd_vel_ready(self):
        self.cmd_vel = None
        while self.cmd_vel is None and not rospy.is_shutdown():
            try:
                self.cmd_vel = rospy.wait_for_message(self.pr2_cmd_vel_topic, Twist, timeout=1.0)
                rospy.logdebug("Current "+self.pr2_cmd_vel_topic+" READY=>" + str(self.cmd_vel))

            except:
                rospy.logerr("Current "+self.pr2_cmd_vel_topic+" not ready yet, retrying for getting "+self.pr2_cmd_vel_topic+"")
        return self.cmd_vel
    
    def _cmd_vel_callback(self, msg):
        print("TEST")
        self.cmd_vel = msg
        rospy.loginfo(self.cmd_vel)
        self.move()
        


    def move(self):
        """
        linear: 
        x: 0.0
        y: 0.0
        z: 0.0
        angular: 
        x: 0.0
        y: 0.0
        z: 0.0
        """
        #self.cmd_vel.
        x_lin = self.cmd_vel.linear.x
        y_lin = self.cmd_vel.linear.y
        z_lin = self.cmd_vel.linear.z
        z_angular = self.cmd_vel.angular.z

        if z_angular != 0.0:
            # Pivot
            self.pr2_mover.move(direction_angle=0.0, lin_speed=z_angular, pivot=True)
        else:
            if x_lin == 0.0 and y_lin == 0.0:
                self.pr2_mover.move(direction_angle=0.0, lin_speed=0.0, pivot=False, stop=True)
            else:
                alfa = math.atan2(y_lin,x_lin)
                magnitude = math.sqrt(math.pow(x_lin,2)+math.pow(y_lin,2))
                self.pr2_mover.move(direction_angle=alfa, lin_speed=magnitude, pivot=False)
        
        self.pr2_mover.move_torso(z_lin)

    def loop(self):
        rospy.spin()


    

if __name__ == "__main__":
    rospy.init_node('pr2_mover_demo', anonymous=True, log_level=rospy.DEBUG)
    pr2_teleop_bridge = Pr2TeleopBridge()
    pr2_teleop_bridge.loop()

