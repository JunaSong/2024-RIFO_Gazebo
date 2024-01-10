#include "func.cpp"

using namespace std;
using namespace Eigen;

int op_mode;

int main(int argc, char **argv){
	ros::init(argc, argv, "jointcmd_publishing_node");
	ros::NodeHandle nh;

	ros::Publisher arm_command_pub = nh.advertise<std_msgs::Float32MultiArray>("/rrbot/ArmCmd",100);

	std_msgs::Float32MultiArray joint_command_msg;

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
			th_cmd[0] = th1*PI/180;
			th_cmd[1] = th2*PI/180;
			th_cmd[2] = th3*PI/180;
		}
		else if(op_mode == 2) // EE pose input
		{
			char s;
			cout << "Enter the target position and the solution type (u for upper, l for lower) : " << '\n';
			cin >> x >> y >> z >> s;
			if(s == 'u')
				up = true;
			else if(s == 'l')
				up = false;
			else{
				cout << "Wrong solution! Type it again." << '\n';
			}

			try{
				Inverse_K(x, y, z, up, th_cmd);
			}
			catch(...){
				cout << "Out of Joint Configuration!" << '\n';
				continue;
			}
		}
		else{
			cout << "try again!\n\n";
            continue;
		}

        joint_command_msg.data.clear();
		for(int i = 0; i < DoF; i++) {
			joint_command_msg.data.push_back(th_cmd[i]);
		}
		arm_command_pub.publish(joint_command_msg);

        // cout << "Target Joint Values :" << th_cmd[0]*180/PI << '\t' << th_cmd[1]*180/PI << '\t' << th_cmd[2]*180/PI << '\n';
		ros::spinOnce();
	}
    return 0;
}
