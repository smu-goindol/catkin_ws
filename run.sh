CATKIN_DIR=/root/catkin_ws

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

cd $CATKIN_DIR

source /opt/ros/noetic/setup.bash
source ./devel/setup.bash

catkin_make
roslaunch assignment_1 parking.launch
