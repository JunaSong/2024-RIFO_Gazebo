#include "func.cpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

int op_mode;

class JointCommandPublisher : public rclcpp::Node
{
public:
    JointCommandPublisher()
    : Node("jointcmd_publishing_node")
    {
        arm_command_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/rrbot/ArmCmd", 100);

        // messages
        joint_command_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();

        // set up timer with 100Hz rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&JointCommandPublisher::timerCallback, this));

        // initialize variables
        op_mode = 0;
        th1 = 0.0; th2 = 0.0; th3 = 0.0; x = 0.0; y = 0.0; z = 0.0;
        up = false;
        for (int i = 0; i < DoF; ++i)
        {
            th_cmd[i] = 0.0;
        }
    }

private:
    void timerCallback()
    {
        cout << "Choose control mode ( 1: joint position / 2: EE pose ) : " << '\n';
        cin >> op_mode;

        if (op_mode == 1) // joint position input
        {
            cout << "Enter the target joint position : " << '\n';
            cin >> th1 >> th2 >> th3;
            if (abs(th1) > 360 or abs(th2) > 360 or abs(th3) > 360)
            {
                cout << "Out of Joint Configuration!" << '\n';
                return;
            }
            th_cmd[0] = th1 * PI / 180;
            th_cmd[1] = th2 * PI / 180;
            th_cmd[2] = th3 * PI / 180;
        }
        else if (op_mode == 2) // EE pose input
        {
            char s;
            cout << "Enter the target position and the solution type (u for upper, l for lower) : " << '\n';
            cin >> x >> y >> z >> s;
            if (s == 'u')
                up = true;
            else if (s == 'l')
                up = false;
            else
            {
                cout << "Wrong solution! Type it again." << '\n';
                return;
            }

            try
            {
                Inverse_K(x, y, z, up, th_cmd);
            }
            catch (...)
            {
                cout << "Out of Joint Configuration!" << '\n';
                return;
            }
        }
        else
        {
            cout << "Try again!\n\n";
            return;
        }

        joint_command_msg->data.clear();
        for (int i = 0; i < DoF; i++)
        {
            joint_command_msg->data.push_back(th_cmd[i]);
        }
        arm_command_pub->publish(*joint_command_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_command_pub;
    std::shared_ptr<std_msgs::msg::Float32MultiArray>joint_command_msg;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointCommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
