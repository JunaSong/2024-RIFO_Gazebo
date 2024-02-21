#ifndef _RRBOT_ROS2_PLUGIN_HH_
#define _RRBOT_ROS2_PLUGIN_HH_

#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

namespace gazebo
{
    class rrbot_ros2_plugin : public ModelPlugin
    {
        // Constructor
        public: rrbot_ros2_plugin() {
            int argc = 0;
            //rclcpp::init(argc, nullptr);
        }

        // Destructor
        public: ~rrbot_ros2_plugin() {}

        // Pointer to subscriber
        private: rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_torque;
        // Pointer to publisher
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointpos;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointvel;

        // Pointer to model
        private: physics::ModelPtr model;

        // Pointer to the joint controller
        private: physics::JointControllerPtr joint1_Controller_;
        private: physics::JointControllerPtr joint2_Controller_;
        private: physics::JointControllerPtr joint3_Controller_;

        // Pointer to the joint
        private: physics::JointPtr joint1_;
        private: physics::JointPtr joint2_;
        private: physics::JointPtr joint3_;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        // Start node
        private: rclcpp::Node::SharedPtr node;


        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;

            // Initialize node
            node = rclcpp::Node::make_shared("rrbot_ros2_plugin");

            // Print messages
            RCLCPP_INFO(this->node->get_logger(), "rrbot_ros2_plugin run");
            std::cerr << "The rrbot_ros2 plugin is attached to the model ["
                      << this->model->GetName() << "]\n";

            std::string robot_namespace = "/" + this->model->GetName() + "/";

            // Store the joint
            this->joint1_ = this->model->GetJoint("rrbot_ros2::joint1");
            this->joint2_ = this->model->GetJoint("rrbot_ros2::joint2");
            this->joint3_ = this->model->GetJoint("rrbot_ros2::joint3");

            // Store the joint Controller to control Joint
            this->joint1_Controller_ = this->model->GetJointController();
            this->joint2_Controller_ = this->model->GetJointController();
            this->joint3_Controller_ = this->model->GetJointController();

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
            //qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            //qos.transient_local();

            // Create subscriber
            this->sub_torque = this->node->create_subscription<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"Torque_sim",
                qos,
                std::bind(&rrbot_ros2_plugin::ROSCallbackTorque_sim, this, std::placeholders::_1));

            // Create publisher
            this->pub_jointpos = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"JointPos_sim", qos);
            this->pub_jointvel = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"JointVel_sim", qos);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&rrbot_ros2_plugin::OnUpdate, this));
        }

        public: void OnUpdate()
        {
            // Publish joint position
            std_msgs::msg::Float32MultiArray JointPos;
            JointPos.data.clear();
            JointPos.data.push_back(this->joint1_->Position(1));
            JointPos.data.push_back(this->joint2_->Position(1));
            JointPos.data.push_back(this->joint3_->Position(1));
            pub_jointpos->publish(JointPos);

            // Publish joint velocity
            std_msgs::msg::Float32MultiArray JointVel;
            JointVel.data.clear();
            JointVel.data.push_back(this->joint1_->GetVelocity(1));
            JointVel.data.push_back(this->joint2_->GetVelocity(1));
            JointVel.data.push_back(this->joint3_->GetVelocity(1));
            pub_jointvel->publish(JointVel);

            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(this->node);
            executor.spin_once();
        }

        void ROSCallbackTorque_sim(const std_msgs::msg::Float32MultiArray::ConstSharedPtr torque)
        {
            this->joint1_->SetForce(0, torque->data[0]);
            this->joint2_->SetForce(0, torque->data[1]);
            this->joint3_->SetForce(0, torque->data[2]);
        }
    };
    
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(rrbot_ros2_plugin);
}
#endif
