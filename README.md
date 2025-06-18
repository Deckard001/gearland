# Create a new workspace directory
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Create the package with necessary dependencies
# Make sure you have these installed: sudo apt-get install ros-<distro>-ros-control ros-<distro>-ros-controllers ros-<distro>-gazebo-ros-control
catkin_create_pkg quadruped_control rospy controller_manager joint_state_controller joint_trajectory_controller gazebo_ros_control

# Create folders for our files
cd quadruped_control
mkdir launch urdf config scripts























lsb_release -a

Confirm that you have correctly added the ROS package repository to your system's software sources. The /etc/apt/sources.list.d/ros-latest.list file should contain the following line:

deb http://packages.ros.org/ros/ubuntu bionic main


cat /etc/apt/sources.list.d/ros-latest.list

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-ros-controllers



OTRO CODIGO

source /opt/ros/melodic/setup.bash
