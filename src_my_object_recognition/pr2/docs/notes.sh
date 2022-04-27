rosrun actionlib_tools axclient.py /tuck_arms


roslaunch moveit_setup_assistant setup_assistant.launch

# Planning Offline
roslaunch pr2_tc_movit demo.launch

#### Installs for extended object detection
sudo apt-get install libzbar-dev libzbar0
sudo apt install ros-noetic-usb-cam

git clone https://github.com/Extended-Object-Detection-ROS/extended_object_detection.git
git clone https://github.com/Extended-Object-Detection-ROS/example_data
catkin_make
source devel/setup.bash
rospack profile


####################
# Object recognition
####################
/static_cam_pub