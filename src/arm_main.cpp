#include <iostream>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
//--------------------MSG------------------------//
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "func.cpp"

using namespace std;
using namespace Eigen;

int op_mode;

void msgCallbackP(const sensor_msgs::JointState::ConstPtr& msg)
{
	for (int i = 0; i < DoF; i++)
	{
		th_act[i] = msg->position[i];
	}
    first_callback = true;
}

void msgCallbackArmCmd_sim(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for (int i = 0; i < DoF; i++)
    {
        th_cmd[i] = msg->data[i];
    }
    traj_init = true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joint_publishing_node");
	ros::NodeHandle nh;

	ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 10);
	ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>("/rrbot/joint2_position_controller/command", 10);
	ros::Publisher joint3_pub = nh.advertise<std_msgs::Float64>("/rrbot/joint3_position_controller/command", 10);
	ros::Publisher torque_pub = nh.advertise<std_msgs::Float64MultiArray>("Torque",10);
	ros::Subscriber sub_joint_angle = nh.subscribe("/rrbot/joint_states", 100, msgCallbackP);
    ros::Subscriber sub_joint_cmd = nh.subscribe("/rrbot/ArmCmd_sim", 100, msgCallbackArmCmd_sim);

	ros::Rate loop_rate(100); 
	ros::spinOnce();

    // messages
	std_msgs::Float64 joint1_msg, joint2_msg, joint3_msg;

	while(ros::ok())
	{
		if(cmd_mode == 0) // Default mode
        {
            for (int i = 0; i < DoF; i++) {
                TargetPos[i] = th_cmd[i];
            }
        }

        else if(cmd_mode == 1)  // cmd angle mode
        {
            if(traj_init == true) // set when cmd angle come in, trajectory generated
            {
                for(int i = 0; i < DoF; i++) {
                    th_ini[i] = th_act[i];
                }
                Traj_joint(th_ini, th_cmd, th_out);
                traj_cnt = 0;
                traj_init = false;
            }
            else
            {
				if(traj_cnt < th_out.rows()) // this portion must not run when no cmd
				{
					for (int i = 0; i < DoF; i++){
						TargetPos[i] = th_out(traj_cnt, i);
					}
					traj_cnt++;
				}
            }
        }

        // PID_controller(TargetPos, th_act, TargetTor);

        // torque_msg.data.clear();
        // for (int i = 0; i < DoF; i++){
        //     torque_msg.data.push_back(TargetTor[i]);
        // }
        
        // torque_pub.publish(torque_msg);

        if(first_callback == true)
        {
            joint1_msg.data = TargetPos[0];
            joint2_msg.data = TargetPos[1];
            joint3_msg.data = TargetPos[2];

            joint1_pub.publish(joint1_msg);
            joint2_pub.publish(joint2_msg);
            joint3_pub.publish(joint3_msg);
        }

        ros::spinOnce();
	}
}
