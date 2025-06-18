# Publish all joint states -----------------------------------
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

# Joint Trajectory Controller -------------------------------
# This is the main controller that accepts position commands
joint_trajectory_controller:
  type: "position_controllers/JointTrajectoryController"
  joints:
    - front_right_hip_joint
    - front_right_knee_joint
    - front_left_hip_joint
    - front_left_knee_joint
    - rear_right_hip_joint
    - rear_right_knee_joint
    - rear_left_hip_joint
    - rear_left_knee_joint

  gains: # Optional, but good practice for stability
    front_right_hip_joint:  {p: 100.0, i: 0.01, d: 10.0}
    front_right_knee_joint: {p: 100.0, i: 0.01, d: 10.0}
    front_left_hip_joint:   {p: 100.0, i: 0.01, d: 10.0}
    front_left_knee_joint:  {p: 100.0, i: 0.01, d: 10.0}
    rear_right_hip_joint:   {p: 100.0, i: 0.01, d: 10.0}
    rear_right_knee_joint:  {p: 100.0, i: 0.01, d: 10.0}
    rear_left_hip_joint:    {p: 100.0, i: 0.01, d: 10.0}
    rear_left_knee_joint:   {p: 100.0, i: 0.01, d: 10.0}









"<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>"





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
