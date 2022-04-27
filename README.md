REFERENCES:

https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/

https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation

____________________________________________________________________________________________________________________________________

STEP BY STEP:

CREATE THE CATKIN_WS FOLDER AND INCLUDE THE SRC SUBFOLDER, AFTER THAT:

$ cd ~/catkin_ws/src/

sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git

$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git

$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git

$ sudo apt install ros-noetic-ros-control* && ros-noetic-control* 

$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
  
$ sudo apt install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench*

$ sudo apt install ros-noetic-robotis-manipulator

source /opt/ros/noetic/setup.bash

$ cd ~/catkin_ws && catkin_make 


____________________________________________________________________________________________________________________________________

TYPE FOR EACH NEW TERMINAL:

source devel/setup.bash && export TURTLEBOT3_MODEL=waffle_pi

____________________________________________________________________________________________________________________________________

FOR MANIPULATION:

- IN THE FILE:
~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/launch/turtlebot3_manipulation_gazebo.launch
INCLUDE BELOW LINE 10 - ADD THE FOLLOWING CODE WITHIN < >:
  arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/
CHANGE THE LINE 24:
    args="-urdf -param robot_description -model robot -x -3.0 -y 1.0 -Y 0.0 -J joint1 0.0 -J joint2 0.0 -J joint3 0.0 -J joint4 0.0 -J gripper 0.0 -J gripper_sub 0.0"/>
SAVE AND CLOSE THE FILE. 

TERMINAL 1: REMENBER TO PRESS PLAY IN THE GAZEBO SIMULATION

roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

TERMINAL 2:

roslaunch turtlebot3_manipulation_moveit_config move_group.launch

TERMINAL 3:

roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch

OR

roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch

TERMINAL 4:

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


____________________________________________________________________________________________________________________________________

FOR MAPING, LOCALIZATION AND NAVIGATION (WITH MANIPULATOR) 

- IN THE FILE:
~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/launch/turtlebot3_manipulation_gazebo.launch
INCLUDE BELOW LINE 10 - ADD THE FOLLOWING CODE WITHIN < >:
  arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/
CHANGE THE LINE 24:
    args="-urdf -param robot_description -model robot -x -3.0 -y 1.0 -Y 0.0 -J joint1 0.0 -J joint2 0.0 -J joint3 0.0 -J joint4 0.0 -J gripper 0.0 -J gripper_sub 0.0"/>
SAVE AND CLOSE THE FILE.

TERMINAL 1: REMENBER TO PRESS PLAY IN THE GAZEBO SIMULATION
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

TERMINAL 2:
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

TERMINAL 3:
roslaunch turtlebot3_manipulation_slam slam.launch

AFTER MOVE THE ROBOT IN THE ROOM AND CONCLUDE THE MAP
STOP TERMINAL 2 AND SAVE THE MAP:
rosrun map_server map_saver -f ~/map
THE MAP IS SAVED ON THE ROOT ~/ WITH THE NAME MAP

STOP TERMINAL 3 AND TYPE THE FOLLOWING
roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=$HOME/map.yaml

IN RVIZ GIVE THE INITIAL POSE WITH THE 2D POSE ESTIMATION
IN THE TERMINAL 2, WITH THE TELEOP YOU CAN IMPROVE THE REAL POSITION WHILE NAVIGATE. 
aFTER, STOP TERMINAL 2 AND RUN THE 2D NAV GOAL
TO GIVE THE DESIRED FINAL POSITION


____________________________________________________________________________________________________________________________________

FOR MAPING, LOCALIZATION AND NAVIGATION (WITHOUT MANIPULATOR) 

TERMINAL 1:
roslaunch turtlebot3_gazebo turtlebot3_house.launch

TERMINAL 2:
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

TERMINAL 3:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

AFTER MOVE THE ROBOT IN THE ROOM AND CONCLUDE THE MAP
STOP TERMINAL 2 AND SAVE THE MAP:
rosrun map_server map_saver -f ~/map
THE MAP IS SAVED ON THE ROOT ~/ WITH THE NAME MAP

STOP TERMINAL 3 AND TYPE THE FOLLOWING
roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=$HOME/map.yaml

IN RVIZ GIVE THE INITIAL POSE WITH THE 2D POSE ESTIMATION
IN THE TERMINAL 2, WITH THE TELEOP YOU CAN IMPROVE THE REAL POSITION WHILE NAVIGATE. 
aFTER, STOP TERMINAL 2 AND RUN THE 2D NAV GOAL
TO GIVE THE DESIRED FINAL POSITION

____________________________________________________________________________________________________________________________________


THE FOLLOWING GIVES A FULL PATH TO SIMULATE AN OPERATION THAT NAVIGATE TO A DESIRED POSE,
PICK THE OBJECT, MOVE TO ANOTHER POSE AND PLACE THE OBJECT.
IN THE MOBILE_MANIPULATOR.PY, CHANGE THE VALUES OF DESIRED POSE.

cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && export TURTLEBOT3_MODEL=waffle_pi

roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=$HOME/map.yaml

roslaunch turtlebot3_manipulation_moveit_config move_group.launch

rosrun my_package mobile_manipulator.py

____________________________________________________________________________________________________________________________________

CHANGE THE TURTLEBOT3 WAFFLE-PI CAMERA TO RGBD

REFERENCE - http://gazebosim.org/tutorials?tut=ros_gzplugins#DepthCamera

COPY THE CAMERA PLUGIN CONTENT (LINES 147-187) OF THE FILE /my_package/urdf/turtlebot3_waffle_pi_for_open_manipulator.gazebo.xacro
AND REPLACE IN THE FOLLOWING (MAKE A BACKUP OF THE ORIGINAL CONTENT IN CASE OF CHANGE BACK TO RGB):

sudo gedit /opt/ros/noetic/share/turtlebot3_description/urdf/turtlebot3_waffle_pi_for_open_manipulator.gazebo.xacro

____________________________________________________________________________________________________________________________________

FOR USING WITH REAL RGBD CAMERA REALSENSE: 
https://github.com/IntelRealSense/realsense-ros

sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

SIMPLE LAUNCH: roslaunch realsense2_camera rs_camera.launch
POINTCLOUDS LAUNCH: roslaunch realsense2_camera demo_pointcloud.launch

____________________________________________________________________________________________________________________________________

RTABMAP - VISUAL SLAM:

sudo apt install ros-noetic-rtabmap-ros

1- SLAM:

touch ~/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_slam/launch/slam_rtab.launch

chmod +x ~/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_slam/launch/slam_rtab.launch

copy the content of "/turtlebot_nav_manip/src/my_package/launch/slam_rtab.launch" in the following:
gedit ~/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_slam/launch/slam_rtab.launch


FOR EACH TERMINAL: cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && export TURTLEBOT3_MODEL=waffle_pi

TERMINAL 1 (remember to play gazebo): roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

TERMINAL 2: roslaunch turtlebot3_manipulation_slam slam_rtab.launch

crtl+C in the TERMINAL 2 and after that: rtabmap-databaseViewer ~/rtabmap.db

2- LOCALIZATION+NAVIGATION:

touch ~/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_slam/launch/nav_rtab.launch

chmod +x ~/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_slam/launch/nav_rtab.launch

copy the content of "/turtlebot_nav_manip/src/my_package/launch/nav_rtab.launch" in the following:
gedit ~/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_slam/launch/nav_rtab.launch


FOR EACH TERMINAL: cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && export TURTLEBOT3_MODEL=waffle_pi

TERMINAL 1 (remember to play gazebo): roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

TERMINAL 2 (add display -> rtabmap_ros -> MapCloud, after click in Download map): 
roslaunch turtlebot3_manipulation_slam nav_rtab.launch localization:=true

____________________________________________________________________________________________________________________________________

MY_OBJECT_RECOGNITION: SIMULATION OF PR2 AND YOLO 

DOWNLOAD THE CONTENT OF THE src_my_object_recognition FOLDER AND PASTE INSIDE YOUR CATKIN_WS/SRC

COMPILE THE PACKAGES WITH catkin_make

TERMINAL 1: roslaunch pr2_tc_gazebo main_elephant_person.launch

TERMINAL 2: roslaunch my_object_recognition_pkg darknet_ros_3d.launch

Extract Position (must be improved):
- rosrun my_object_recognition_pkg yolo_3d_data_extraction.py


