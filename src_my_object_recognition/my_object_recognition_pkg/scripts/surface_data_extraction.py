#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy

from visualization_msgs.msg import Marker

class SurfaceObjectFilter:

   def __init__(self, table_height_init=0.8, error_height=0.2):

      self._rate = rospy.Rate(5)

      self.table_height = table_height_init
      self._error_height = error_height
      self.surface_dict = {}
      self.surface_topic = "/surface_objects"
      self._check_surface_ready()
      rospy.Subscriber(self.surface_topic, Marker, self.surface_callback)
     
      rospy.loginfo('Ready to detect Surfaces!')

   def _check_surface_ready(self):
      self._surface_data = None
      while self._surface_data is None and not rospy.is_shutdown():
         try:
               self._surface_data = rospy.wait_for_message(self.surface_topic, Marker, timeout=1.0)
               rospy.logdebug("Current "+self.surface_topic+" READY=>" + str(self._surface_data))

         except:
               rospy.logerr("Current "+self.surface_topic+" not ready yet, retrying.")

   def update_table_height(self,new_table_height):
      self.table_height = new_table_height

   def look_for_table_surface(self, z_value):
      """

      """
      delta_min = z_value - self._error_height
      delta_max = z_value + self._error_height
      is_the_table = delta_min < self.table_height < delta_max

      return is_the_table

   def surface_callback(self, msg):

      name = msg.ns
      surface_pose = msg.pose

      if "surface_" in name and not "_axes" in name:
         # We check the heigh in z to see if its the table
         if self.look_for_table_surface(msg.pose.position.z):
            if name in self.surface_dict:
               rospy.loginfo("This surface was alreday found")
            else:
               self.surface_dict[name] = surface_pose
               rospy.loginfo("Found New Surface=")
      else:
         rospy.logdebug("Surface Object Not found "+str(name))


   def get_surface_dict_detected(self):
      return self.surface_dict

   def run(self):


      while not rospy.is_shutdown():
         table_surfaces_detected = self.get_surface_dict_detected()
         rospy.loginfo(str(table_surfaces_detected))
         self._rate.sleep()



if __name__ == '__main__':
   rospy.init_node('surface_data_extract_node', log_level=rospy.INFO)

   try:
      SurfaceObjectFilter().run()
   except KeyboardInterrupt:
      rospy.loginfo('Shutting down')

