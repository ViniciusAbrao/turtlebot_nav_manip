cmake_minimum_required(VERSION 2.8.3)
project(gb_visual_detection_3d_msgs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
 FILES
 BoundingBoxes3d.msg
 BoundingBox3d.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
