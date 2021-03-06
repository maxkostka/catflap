#! /bin/bash
sleep 1
WLAN_IP="$(ip a | sed -rn '/: 'wlan0':.*state UP/{N;N;s/.*inet (\S*).*/\1/p}')"
# wait for wlan0 to come up for max 180s
counter=180
while [ -z "$WLAN_IP" ] && [ $counter -gt 1 ]
do
    echo "wlan0 down. checking for $counter more seconds"
    sleep 5
    WLAN_IP="$(ip a | sed -rn '/: 'wlan0':.*state UP/{N;N;s/.*inet (\S*).*/\1/p}')"
    ((counter-=5))
done
if [ -n "$WLAN_IP" ]
then
    echo "wlan0 has a valid ip address $WLAN_IP."
else
    echo "wlan0 has no valid ip."
fi

echo "exporting ROS env variables"
export ROS_ROOT=/opt/ros/indigo/share/ros
export ROS_PACKAGE_PATH=/home/max/projects/catflap/ros_catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
export ROS_MASTER_URI=http://raspberry:11311
export ROSLISP_PACKAGE_DIRECTORIES=/home/max/projects/catflap/ros_catkin_ws/devel/share/common-lisp
export ROS_DISTRO=indigo
export ROS_ETC_DIR=/opt/ros/indigo/etc/ros
echo "ROS_ROOT="$ROS_ROOT
echo "ROS_PACKAGE="$ROS_PACKAGE
echo "ROS_MASTER_URI="$ROS_MASTER_URI
echo "ROSLISP_PACKAGE_DIRECTORIES="$ROSLISP_PACKAGE_DIRECTORIES
echo "ROS_ETC_DIR="$ROS_ETC_DIR
echo "ROS_LOG="$ROS_LOG
echo "sourcing ros setup files"
sleep 3
source /opt/ros/indigo/setup.bash
sleep 3
source /home/max/projects/catflap/ros_catkin_ws/devel/setup.bash
echo "preparing virtualenv"
export WORKON_HOME=/home/max/.virtualenvs
sleep 3
source /usr/local/bin/virtualenvwrapper.sh
echo "WORKON_HOME="$WORKON_HOME
#echo "switching to virtual environment cv"
sleep 3
workon cv
echo "try launching catflap"
sleep 3
roslaunch /home/max/projects/catflap/ros_catkin_ws/src/catflap_training.launch
echo "ros exit - startup.sh will be closed"
