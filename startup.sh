#! /bin/bash
export ROS_ROOT=/opt/ros/indigo/share/ros
export ROS_PACKAGE_PATH=/home/max/projects/catflap/ros_catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
export ROS_MASTER_URI=http://raspberry:11311
export ROSLISP_PACKAGE_DIRECTORIES=/home/max/projects/catflap/ros_catkin_ws/devel/share/common-lisp
export ROS_DISTRO=indigo
export ROS_ETC_DIR=/opt/ros/indigo/etc/ros
source /opt/ros/indigo/setup.bash
source /home/max/projects/catflap/ros_catkin_ws/devel/setup.bash
export WORKON_HOME=/home/max/.virtualenvs
source /usr/local/bin/virtualenvwrapper.sh
echo switching to virtual environment cv
workon cv
roslaunch /home/max/projects/catflap/ros_catkin_ws/src/catflap_training.launch

