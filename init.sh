apt install curl
apt install lsb_release

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

apt update
apt install ros-noetic-desktop-full
pip install -r requirements.txt

echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "alias cm='cd ~/catkin_ws && catkin_make'" >> ~/.bashrc
echo "alias run='cd ~/catkin_ws && roslaunch assignment_1 parking.launch'" >> ~/.bashrc

source ~/.bashrc
