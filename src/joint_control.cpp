#include <iostream>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
//--------------------MSG------------------------//
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "func.cpp"

using namespace std;
using namespace Eigen;

int op_mode;

void msgCallbackP(const sensor_msgs::JointState::ConstPtr& msg)
{
    th1_i = msg->position[0];
	th2_i = msg->position[1];
	th3_i = msg->position[2];
    first_callback = true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joint_publishing_node");
	ros::NodeHandle nh;

	ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 10);
	ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>("/rrbot/joint2_position_controller/command", 10);
	ros::Publisher joint3_pub = nh.advertise<std_msgs::Float64>("/rrbot/joint3_position_controller/command", 10);
	ros::Publisher torque_pub = nh.advertise<std_msgs::Float64MultiArray>("Torque",10);
	ros::Subscriber sub_joint_angle = nh.subscribe("/rrbot/joint_states", 100, msgCallbackP);

	ros::Rate loop_rate(100); 
	ros::spinOnce();

	while(ros::ok())
	{
		cout << "Choose control mode ( 1: joint position / 2: EE pose ) : " << '\n';
		cin >> op_mode;

		if(op_mode == 1) // joint position input
		{
			cout << "Enter the target joint position : " << '\n';
			cin >> th1 >> th2 >> th3;
			if (abs(th1) > 360 or abs(th2) > 360 or abs(th3) > 360){
				cout << "Out of Joint Configuration!" << '\n';
				continue;
			}
			th1 *= PI/180;
			th2 *= PI/180;
			th3 *= PI/180;

			Forward_K(th1,th2,th3);
		}
		else if(op_mode == 2) // EE pose input
		{
			char s;
			cout << "Enter the target position and the solution type (U for upper, L for lower) : " << '\n';
			cin >> x >> y >> z >> s;
			if(s == 'U')
				up = true;
			else if(s == 'L')
				up = false;
			else{
				cout << "Wrong solution! Type it again." << '\n';
			}

			try{
				Inverse_K(x, y, z, up);
			}
			catch(...){
				cout << "Out of Joint Configuration!" << '\n';
				continue;
			}
		}
		const int step = 1000;
		bool initial_state_read = true;

		std_msgs::Float64 joint1_msg, joint2_msg, joint3_msg;

		if (first_callback){
			if (initial_state_read){
				initial_state_read = false;
				cout << "initial_pose : " << th1_i << '\t' << th2_i << '\t' << th3_i << '\n';
				for (int i = 0; i < step; i++){
					joint1_msg.data = (i + 1) * (th1 - th1_i) / step + th1_i;
					joint2_msg.data = (i + 1) * (th2 - th2_i) / step + th2_i;
					joint3_msg.data = (i + 1) * (th3 - th3_i) / step + th3_i;

					joint1_pub.publish(joint1_msg);
					joint2_pub.publish(joint2_msg);
					joint3_pub.publish(joint3_msg);

					loop_rate.sleep();
				}
			}
		}
		else{
			cout << "Waiting Joint States..." << '\n';
		}
		ros::spinOnce();

	}
}
