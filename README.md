## 2024 Robotics Innovatory Freshman Orientation

## Quick Start

Launch Robot:

    ros2 launch ros2_control_demo_bringup rrbot.launch.py

Run Control Node:

    ros2 run arm_control main

Run Command Node:

    ros2 run arm_control arm_command


Example of Moving Joints:

    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 0.5
    - 0.5
    - 0.5"

# References

ROS2 Control Demos
