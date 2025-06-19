<?xml version="1.0"?>
<model>
  <name>My Hexapod</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf> <author>
    <name>Your Name</name>
    <email>your@email.com</email>
  </author>

  <description>
    A model of a six-legged hexapod robot.
  </description>
</model>
















install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


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
