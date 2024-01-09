#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
using Eigen::VectorXf;


#define PI 3.14159
#define SAMPLING_TIME = 0.001

float L1 = 0.5, L2 = 1, L3 = 1;
float th1_i, th2_i, th3_i, 
	  th1, th2, th3,
	  x, y, z;
bool first_callback = true, up = true;
//--------------DH param-------------------//
int DoF = 3;

//--------------PID gain-------------------//
float TargetTor[3] = {}; 
float Kp[3] = {},
      Ki[3] = {},
      Kd[3] = {};

//--------------Functions-------------------//
Matrix4f Rz(float th){
	Matrix4f Rz_;
	Rz_ << cos(th), -sin(th), 0, 0,
           sin(th), cos(th), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
	return Rz_;
}

Matrix4f Rx(float th){
	Matrix4f Rx_;
	Rx_ <<  1, 0, 0, 0,
        	0, cos(th), -sin(th), 0,
            0, sin(th), cos(th), 0,
            0, 0, 0, 1;
	return Rx_;
}

Matrix4f Tx(float a){
	Matrix4f Tx_;
	Tx_ <<  1, 0, 0, a,
        	0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
	return Tx_;
}

Matrix4f Tz(float d){
	Matrix4f Tz_;
	Tz_ <<  1, 0, 0, 0,
        	0, 1, 0, 0,
            0, 0, 1, d,
            0, 0, 0, 1;
	return Tz_;
}

Matrix4f T_cor(float th, float d, float al, float a){
	Matrix4f T_cor_;
	T_cor_ <<  cos(th), -cos(al)*sin(th), sin(al)*sin(th), a*cos(th),
        	   sin(th), cos(al)*cos(th), -sin(al)*cos(th), a*sin(th),
               0, sin(al), cos(al), d,
               0, 0, 0, 1;
	return T_cor_;
}

// input DH, output target value
void Forward_K(float th1, float th2, float th3)
{
	Vector3f theta, d, a, alpha;
	theta << th1, th2 + PI/2, th3;
	d << L1, 0, 0;
	alpha << PI/2, 0, 0;
	a << 0, L2, L3;

	Matrix4f T = Matrix4f::Identity();

	for (int i = 0; i < DoF; i++)
	{
		T *= T_cor(theta(i), d(i), alpha(i), a(i));
	}
	
    cout << "Target Position Values :" << T(0,3) << T(1,3) << T(2,3) << '\n';
}

void Inverse_K(float x, float y, float z, bool up_down)
{
	th1 = atan2(y, x);
	if (th1 <= - PI)
		th1 += PI;
	
	float Ld = sqrt(pow(x,2) + pow(y,2) + pow(z - L1,2));

	if(up_down){
		th3 = -(PI - acos((pow(L2,2) + pow(L2,2) - pow(Ld,2)) / (2*L2*L3)));
		th2 = -(atan2(sqrt(pow(x,2) + pow(y,2)), z - L1) + th3/2);
	}
	else{
		th3 = PI - acos((pow(L2,2) + pow(L3,2) - pow(Ld,2)) / (2*L2*L3));
        th2 = -(atan2(sqrt(pow(x,2) + pow(y,2)), z - L1) + th3/2);
	}
    cout << ("Target Joint Values :", th1*180/PI, th2*180/PI, th3*180/PI) << '\n';
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
