#include "rrbot2_func.cpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

class JointPublishingNode : public rclcpp::Node
{
public:

    JointPublishingNode() : Node("rrbot2_main")
    {
        joint_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/rrbot_ros2/Torque_sim", 10);

        targetpos_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/rrbot_ros2/TargetPos", 10);

        sub_joint_angle = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rrbot_ros2/JointPos_sim", 10, 
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < DoF; i++)
                {
                armjointp[i] = msg->data[i];
                }
                first_callback = true;});

        sub_joint_velocity = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rrbot_ros2/JointVel_sim", 10, 
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < DoF; i++)
                {
                armjointv[i] = msg->data[i];
                }
                first_callback = true;});
                 
        sub_joint_cmd = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rrbot_ros2/ArmCmd", 10, 
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < DoF; i++)
                {
                armjointp_cmd[i] = msg->data[i];
                }
                traj_init = true;});

        timer_ = this->create_wall_timer(10ms, std::bind(&JointPublishingNode::run, this));
    }

private:
    void run()
    {

        Forward_K(armjointp_cmd, T03);

        if (traj_init == true) // set when cmd angle comes in, trajectory generated
        {
            for (int i = 0; i < DoF; i++)
            {
                armjointp_ini[i] = armjointp[i];
            }
            Traj_joint(armjointp_ini, armjointp_cmd, th_out);
            traj_cnt = 0;
            traj_init = false;
        }
        else if (traj_cnt < th_out.rows()) // this portion must not run when no cmd
        {
            for (int i = 0; i < DoF; i++)
            {
                TargetPos[i] = th_out(traj_cnt, i);
            }
            traj_cnt++;
        }
        else
        {
            for (int i = 0; i < DoF; i++)
            {
                TargetPos[i] = armjointp_cmd[i];
            }
        }

        PIDController(TargetPos, armjointp, armjointv, TargetTor);

        if(first_callback == true)
        {
            std_msgs::msg::Float32MultiArray torque_msg;
            torque_msg.data.clear();
            for (int i=0; i < DoF; i++){
            torque_msg.data.push_back(TargetTor[i]);
            }
            joint_pub->publish(torque_msg);

             std_msgs::msg::Float32MultiArray targetpos_msg;
            targetpos_msg.data.clear();
            for (int i=0; i < DoF; i++){
            targetpos_msg.data.push_back(TargetPos[i]);
            }
            targetpos_pub->publish(targetpos_msg);
        }

        cout.precision(3);
        cout << "Target Joint / Pose :" << TargetPos[0] * 180 / PI << setw(6) << TargetPos[1] * 180 / PI << setw(6) << TargetPos[2] * 180 / PI << "  /  ";
        cout << T03(0, 3) << setw(6) << T03(1, 3) << setw(6) << T03(2, 3) << '\n';
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetpos_pub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joint_angle;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joint_velocity;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joint_cmd;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<JointPublishingNode>();
    rclcpp::spin(node1);
    rclcpp::shutdown();
    return 0;
}