#include <iostream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include <eigen3/Eigen/Dense>

//--------------------MSG------------------------//
// #include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std;
using namespace Eigen;
using Eigen::VectorXf;

#define PI 3.14159
#define SAMPLING_TIME 0.01
#define DoF 3

//--------------Command-------------------//
int cmd_mode = 1;
double armjointp[DoF] = {0,};
double armjointp_ini[DoF] = {0,};
double armjointp_cmd[DoF] = {0,};
double armjointv[DoF] = {0,};
MatrixXd T03 = MatrixXd::Identity(4, 4);

bool first_callback = true, up = true;

//--------------DH param-------------------//
float L1 = 0.5, L2 = 1, L3 = 1;
float th1_i, th2_i, th3_i, 
	  th1, th2, th3,
	  x, y, z;

//--------------PID gain-------------------//
double TargetTor[DoF] = {0, };
double TargetPos[DoF] = {0, };
double  Kp[3] = {},
        Ki[3] = {},
        Kd[3] = {};

//--------------Trajectory Planning-------------------//
bool traj_init = false;
int traj_cnt = 0;
MatrixXf th_out = MatrixXf::Zero(1, DoF); // resize later


//--------------Functions-------------------//
Matrix4d T_craig(float th, float d, float al, float a)
{
	Matrix4d T_craig_;
	T_craig_ << cos(th), 			-sin(th), 			0, 			a,
				sin(th)*cos(al), 	cos(th)*cos(al), 	-sin(al), 	-d*sin(al),
				sin(th)*sin(al), 	cos(th)*sin(al), 	cos(al), 	d*cos(al),
				0, 					0, 					0, 			1;
	return T_craig_;
}

// input DH, output target value
void Forward_K(double* th, MatrixXd& T)
{
	double th1 = th[0];
	double th2 = th[1];
	double th3 = th[2];

	Vector4f theta, d, a, alpha;
	theta << th1, th2 - PI/2, th3, 0;
	d << L1, 0, 0, 0;
	alpha << 0, -PI/2, 0, 0;
	a << 0, 0, L2, L3;

	T = Matrix4d::Identity();

	for (int i = 0; i < DoF + 1; i++)
	{
		T *= T_craig(theta(i), d(i), alpha(i), a(i));
	}	
}

void Inverse_K(float x, float y, float z, bool up_down, double* th_out)
{
	double th1, th2, th3;

	th1 = atan2(y, x);
	if (th1 <= - PI)
		th1 += 2*PI;
	
	float Ld = sqrt(pow(x,2) + pow(y,2) + pow(z - L1,2));

	if(up_down){
		th3 = acos( (pow(Ld,2) - pow(L2,2) - pow(L3,2)) / (2*L2*L3) );
		th2 = atan2( sqrt(pow(x,2) + pow(y,2)), z - L1 ) - th3 / 2;
	}
	else{
		th3 = - acos( (pow(Ld,2) - pow(L2,2) - pow(L3,2)) / (2*L2*L3) );
		th2 = atan2( sqrt(pow(x,2) + pow(y,2)), z - L1 ) - th3 / 2;
	}

	th_out[0] = th1;
	th_out[1] = th2;
	th_out[2] = th3;
}

void Traj_joint(double* armjointp_ini, double* armjointp_cmd, MatrixXf& th_out)
{
	// th_out row: 3dof / column: interpolated position
	
	double vel_des = PI/6; // rad/s
	double max_error = 0;
	for (int i = 0; i < DoF; i++) {
		if(max_error < fabs(armjointp_cmd[i] - armjointp_ini[i]))
			max_error = fabs(armjointp_cmd[i] - armjointp_ini[i]);
	}
	
	double Tf = max_error / vel_des; // Use maximum error of angle
	double step = round(Tf / SAMPLING_TIME);
	th_out.resize(step, th_out.cols());

	for (int i = 0; i < DoF; i++)
	{
		RowVectorXf inter_pos = RowVectorXf::LinSpaced(step, armjointp_ini[i], armjointp_cmd[i]);
		th_out.block(0, i, step, 1) = inter_pos.transpose();
	}
}

void PIDController(double* targetpos_, double* currentpos_, double* currentvel_, double* PDtorque_)
{
	double error_old[DoF] = {0,};
    double arm_errorp[DoF] = {0,}; 
    double P_term[DoF] = {0,}; 
    double D_term[DoF] = {0,}; 
	int torque_limit = 100;
	Kp[0] = 100;
	Kp[1] = 200;
	Kp[2] = 80;
	Kd[0] = 10;
	Kd[1] = 10;
	Kd[2] = 10;

	for (int i = 0; i < DoF; i++)
	{
		arm_errorp[i] = targetpos_[i] - currentpos_[i];

		P_term[i] = Kp[i]*arm_errorp[i];
		D_term[i] = Kd[i]*(0 - currentvel_[i]);

		PDtorque_[i] = P_term[i] + D_term[i];

		if(PDtorque_[i] > torque_limit){
			PDtorque_[i] = torque_limit;
		}
		else if(PDtorque_[i] < -torque_limit){
			PDtorque_[i] = -torque_limit;
		}

		error_old[i] = arm_errorp[i];
	}
}