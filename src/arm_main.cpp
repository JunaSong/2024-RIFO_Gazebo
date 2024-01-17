#include "func.cpp"

using namespace std;
using namespace Eigen;

class JointPublishingNode : public rclcpp::Node
{
public:
    JointPublishingNode() : Node("joint_publishing_node")
    {
        joint1_pub = this->create_publisher<std_msgs::msg::Float64>("/rrbot/joint1_position_controller/command", 10);
        joint2_pub = this->create_publisher<std_msgs::msg::Float64>("/rrbot/joint2_position_controller/command", 10);
        joint3_pub = this->create_publisher<std_msgs::msg::Float64>("/rrbot/joint3_position_controller/command", 10);

        sub_joint_angle = this->create_subscription<sensor_msgs::msg::JointState>(
            "/rrbot/joint_states", 100, std::bind(&JointPublishingNode::msgCallbackP, this, std::placeholders::_1));

        sub_joint_cmd = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rrbot/ArmCmd", 100, std::bind(&JointPublishingNode::msgCallbackArmCmd_sim, this, std::placeholders::_1));

        // messages
        joint1_msg = std_msgs::msg::Float64();
        joint2_msg = std_msgs::msg::Float64();
        joint3_msg = std_msgs::msg::Float64();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint1_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint2_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint3_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_angle;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joint_cmd;

    std_msgs::msg::Float64 joint1_msg, joint2_msg, joint3_msg;

    void msgCallbackP(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < DoF; i++)
        {
            th_act[i] = msg->position[i];
        }
        first_callback = true;
    }

    void msgCallbackArmCmd_sim(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        for (int i = 0; i < DoF; i++)
        {
            th_cmd[i] = msg->data[i];
        }
        traj_init = true;
    }

    void run()
    {
        while (rclcpp::ok())
        {
            Forward_K(th_act, T03);

            if (traj_init == true) // set when cmd angle comes in, trajectory generated
            {
                for (int i = 0; i < DoF; i++)
                {
                    th_ini[i] = th_act[i];
                }
                Traj_joint(th_ini, th_cmd, th_out);
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
                    TargetPos[i] = th_cmd[i];
                }
            }

            // PID_controller(TargetPos, th_act, TargetTor);
            // torque_msg.data.clear();
            // for (int i = 0; i < DoF; i++){
            //     torque_msg.data.push_back(TargetTor[i]);
            // }

            // torque_pub.publish(torque_msg);

            if (first_callback == true)
            {
                joint1_msg.data = TargetPos[0];
                joint2_msg.data = TargetPos[1];
                joint3_msg.data = TargetPos[2];

                joint1_pub->publish(joint1_msg);
                joint2_pub->publish(joint2_msg);
                joint3_pub->publish(joint3_msg);
            }

            cout.precision(3);
            // cout << "Target Joint / Pose :" << TargetPos[0] * 180 / PI << setw(6) << TargetPos[1] * 180 / PI << setw(6) << TargetPos[2] * 180 / PI << "  /  ";
            // cout << T03(0, 3) << setw(6) << T03(1, 3) << setw(6) << T03(2, 3) << '\n';

            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPublishingNode>();
    // node->run();
    rclcpp::shutdown();
    return 0;
}
