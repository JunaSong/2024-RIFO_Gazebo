#include "func.cpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;
using std::placeholders::_1;

class JointPublishingNode : public rclcpp::Node
{
public:
    JointPublishingNode() : Node("joint_publishing_node")
    {
        rrbot_joint_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        torque_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/rrbot/Torque_sim", 10);

        sub_joint_angle = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 100, std::bind(&JointPublishingNode::msgCallbackP, this, _1));
        
        sub_joint_cmd = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rrbot/ArmCmd", 100, std::bind(&JointPublishingNode::msgCallbackArmCmd_sim, this, _1));

        timer_ = this->create_wall_timer(10ms, std::bind(&JointPublishingNode::run, this));
    }

private:
    void msgCallbackP(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < DoF; i++)
        {
            CurrentPos[i] = msg->position[i];
        }
        first_callback = true;
    }

    void msgCallbackArmCmd_sim(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        for (int i = 0; i < DoF; i++)
        {
            th_sub[i] = msg->data[i];
        }
        traj_init = true;
    }

    void run()
    {
        auto rrbot_joint_msg = std_msgs::msg::Float64MultiArray();
        // auto torque_msg = std_msgs::msg::Float32MultiArray();

        Forward_K(CurrentPos, T03);
        Forward_K(TargetPos, T03_Tar);

        if (traj_init == true) // set when cmd angle comes in, trajectory generated
        {
            for (int i = 0; i < DoF; i++)
            {
                th_ini[i] = CurrentPos[i];
            }
            Traj_joint(th_ini, th_sub, th_out);
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
                TargetPos[i] = th_sub[i];
            }
        }

        // PID_controller(TargetPos, CurrentPos, TargetTor);
        // torque_msg.data.clear();
        // for (int i = 0; i < DoF; i++){
        //     torque_msg.data.push_back(TargetTor[i]);
        // }
        // torque_pub->publish(torque_msg);

        if (first_callback == true)
        {
            rrbot_joint_msg.data.clear();
            for (int i = 0; i < 3; i++){
                rrbot_joint_msg.data.push_back(TargetPos[i]);
            }
            rrbot_joint_pub->publish(rrbot_joint_msg);
        }

        cout << fixed;
        cout.precision(3);
        cout << "Current Joint / Pose :" << CurrentPos[0] * 180 / PI << setw(8) << CurrentPos[1] * 180 / PI << setw(8) << CurrentPos[2] * 180 / PI << "  /  ";
        cout << T03(0, 3) << setw(8) << T03(1, 3) << setw(8) << T03(2, 3) << '\n';
        cout << "Target Joint  / Pose :" << TargetPos[0] * 180 / PI << setw(8) << TargetPos[1] * 180 / PI << setw(8) << TargetPos[2] * 180 / PI << "  /  ";
        cout << T03_Tar(0, 3) << setw(8) << T03_Tar(1, 3) << setw(8) << T03_Tar(2, 3) << '\n' << '\n';
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rrbot_joint_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_angle;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joint_cmd;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPublishingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
