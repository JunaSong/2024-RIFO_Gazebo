#include <iostream>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
//--------------------MSG------------------------//
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace Eigen;
using Eigen::VectorXf;

#define PI 3.14159
#define SAMPLING_TIME 0.001
#define DoF 3

//--------------Command-------------------//
int cmd_mode = 1;
double th_act[DoF] = {0,};
double th_ini[DoF] = {0,};
double th_cmd[DoF] = {0,};
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
// Matrix4f Rz(float th){
// 	Matrix4f Rz_;
// 	Rz_ << cos(th), -sin(th), 0, 0,
//            sin(th), cos(th), 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
// 	return Rz_;
// }

// Matrix4f Rx(float th){
// 	Matrix4f Rx_;
// 	Rx_ <<  1, 0, 0, 0,
//         	0, cos(th), -sin(th), 0,
//             0, sin(th), cos(th), 0,
//             0, 0, 0, 1;
// 	return Rx_;
// }

// Matrix4f Tx(float a){
// 	Matrix4f Tx_;
// 	Tx_ <<  1, 0, 0, a,
//         	0, 1, 0, 0,
//             0, 0, 1, 0,
//             0, 0, 0, 1;
// 	return Tx_;
// }

// Matrix4f Tz(float d){
// 	Matrix4f Tz_;
// 	Tz_ <<  1, 0, 0, 0,
//         	0, 1, 0, 0,
//             0, 0, 1, d,
//             0, 0, 0, 1;
// 	return Tz_;
// }

// Matrix4d T_cor(float th, float d, float al, float a){
// 	Matrix4d T_cor_;
// 	T_cor_ <<  cos(th), -cos(al)*sin(th), sin(al)*sin(th), a*cos(th),
//         	   sin(th), cos(al)*cos(th), -sin(al)*cos(th), a*sin(th),
//                0, sin(al), cos(al), d,
//                0, 0, 0, 1;
// 	return T_cor_;
// }

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
	// theta << th1, th2 + PI/2, th3;
	// d << L1, 0, 0;
	// alpha << PI/2, 0, 0;
	// a << 0, L2, L3;
	theta << th1, th2 - PI/2, th3, 0;
	d << L1, 0, 0, 0;
	alpha << 0, -PI/2, 0, 0;
	a << 0, 0, L2, L3;

	T = Matrix4d::Identity();

	for (int i = 0; i < DoF + 1; i++)
	{
		T *= T_craig(theta(i), d(i), alpha(i), a(i));
	}
	
    // cout << "Target Position Values :" << T(0,3) << T(1,3) << T(2,3) << '\n';
}

void Inverse_K(float x, float y, float z, bool up_down, double* th_out)
{
	double th1, th2, th3;

	th1 = atan2(y, x);
	if (th1 <= - PI)
		th1 += 2*PI;
	
	float Ld = sqrt(pow(x,2) + pow(y,2) + pow(z - L1,2));

	// if(up_down){
	// 	th3 = -(PI - acos((pow(L2,2) + pow(L2,2) - pow(Ld,2)) / (2*L2*L3)));
	// 	th2 = -(atan2(sqrt(pow(x,2) + pow(y,2)), z - L1) + th3/2);
	// }
	// else{
	// 	th3 = PI - acos((pow(L2,2) + pow(L3,2) - pow(Ld,2)) / (2*L2*L3));
    //     th2 = -(atan2(sqrt(pow(x,2) + pow(y,2)), z - L1) + th3/2);
	// }
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

void Traj_joint(double* th_ini, double* th_cmd, MatrixXf& th_out)
{
	// th_out row: 3dof / column: interpolated position
	
	double vel_des = PI/6; // rad/s
	double max_error = 0;
	for (int i = 0; i < DoF; i++) {
		if(max_error < fabs(th_cmd[i] - th_ini[i]))
			max_error = fabs(th_cmd[i] - th_ini[i]);
	}
	
	double Tf = max_error / vel_des; // Use maximum error of angle
	double step = round(Tf / SAMPLING_TIME);
	th_out.resize(step, th_out.cols());

	for (int i = 0; i < DoF; i++)
	{
		RowVectorXf inter_pos = RowVectorXf::LinSpaced(step, th_ini[i], th_cmd[i]);
		th_out.block(0, i, step, 1) = inter_pos.transpose();
	}
}

// void PID_controller(const Ref<Vector3d> _TargetPos, const Ref<Vector3d> _CurrentPos, Ref<Vector3d> _PDtorque)
// {

//     _PDtorque(0) = Kp*(_TargetPos(0) - _CurrentPos(0)) + Kd*((_TargetPos(0) - _CurrentPos(0))/SAMPLING_TIME);
// }

// void pidControl(int actuator_index, float target, float* error_old)
// {	
// 	float errorlimit;
// 	float error_now;
// 	double PID_control;

//     errorlimit = 4000;
//     error_now = target - th[actuator_index];

//     if(error_now > errorlimit){
//         error_now = errorlimit;
//     }
//     else if(error_now < -errorlimit){
//         error_now = -errorlimit;
//     }
//     P_control[actuator_index] = Kp[actuator_index] * (double)error_now;
//     I_control[actuator_index] = I_control[actuator_index] + Ki[actuator_index] * (double)error_now * sampleing_time;
//     D_control[actuator_index] = Kd[actuator_index] * ((double)error_now - (double)(*error_old)) / sampleing_time;
//     PID_control = P_control[actuator_index] + I_control[actuator_index] + D_control[actuator_index];

//     if(abs(error_now)<(int32_t)1100) // input 10도, output 0.1도
//     {
//         PID_control = 0;
//     }


// 	// if(PID_control > max_torque){
// 	// 	PID_control = max_torque;
// 	// }
// 	// else if(PID_control < (-max_torque)){
// 	// 	PID_control = (-max_torque);
// 	// }

// 	TargetTor[actuator_index] = (int16_t)PID_control;
// 	*error_old = error_now;
// 	PastPos = ActualPos[0];
// }
