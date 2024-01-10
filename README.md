## 2024 Robotics Innovatory Freshman Orientation

## Quick Start

Gazebo:

    roslaunch rifo_gazebo gazebo.launch

ROS Control:

    rosrun rifo_gazebo joint_control


Example of Moving Joints:

    rostopic pub /rrbot/joint2_position_controller/command std_msgs/Float64 "data: -0.9"

# References

Gazebo ROS Demos

* Author: Dave Coleman <davetcoleman@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)
