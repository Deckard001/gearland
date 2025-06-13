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
