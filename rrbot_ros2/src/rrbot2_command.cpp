#include "rrbot2_func.cpp"

class JointCommandPublisher : public rclcpp::Node
{
public:
    JointCommandPublisher()
    : Node("rrbot2_command") //, op_mode(0)
    {
        // Create a publisher on the "/rrbot/ArmCmd" topic
        arm_command_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/rrbot_ros2/ArmCmd", 100);

        // Create a timer to publish the number every 1 second
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&JointCommandPublisher::getJointCmdInput, this));
    }

private:
    void getJointCmdInput()
    {
        while(rclcpp::ok())
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
                armjointp_cmd[0] = th1*PI/180;
                armjointp_cmd[1] = th2*PI/180;
                armjointp_cmd[2] = th3*PI/180;
            }
            else if(op_mode == 2) // EE pose input
            {
                char s;
                cout << "Enter the target position and the solution type (u for upper, l for lower) : " << '\n';
                cin >> x >> y >> z >> s;
                if(s == 'u'){
                    up = true;
                    
                }
                else if(s == 'l'){
                    up = false;
                    
                }
                else{
                    cout << "Wrong solution! Type it again." << '\n';
                    break;
                }

                try{
                    Inverse_K(x, y, z, up, armjointp_cmd);
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
            
            // Create a message and set its data
            std_msgs::msg::Float32MultiArray joint_command_msg;
            joint_command_msg.data.clear();
            for(int i = 0; i < DoF; i++) {
                joint_command_msg.data.push_back(armjointp_cmd[i]);
            }

            // Publish the message
            arm_command_pub->publish(joint_command_msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_command_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    int op_mode;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointCommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}