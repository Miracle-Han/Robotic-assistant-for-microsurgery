#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

using namespace Eigen;

struct DHParameter {
    double theta;  // joint angle
    double alpha;  // link twist
    double a;      // link length
    double d;      // link offset
};

Matrix4d dh_transform(double theta, double alpha, double a, double d) {
    Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
         sin(theta), cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

Matrix4d forward_kinematics(const std::vector<DHParameter>& dh_params) {
    Matrix4d T = Matrix4d::Identity();
    for (const auto& param : dh_params) {
        T *= dh_transform(param.theta, param.alpha, param.a, param.d);
    }
    return T;
}

class ForwardKinematicNode : public rclcpp::Node {
public:
    // Constructor
    ForwardKinematicNode() : Node("forward_kinematic_node") {
        RCLCPP_INFO(this->get_logger(), "Starting ForwardKinematicNode...");
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/joint_angles", 10, std::bind(&ForwardKinematicNode::topic_callback, this, std::placeholders::_1));
    }

    // Callback function
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received joint angles message: '%s'", msg->data.c_str());

        std::istringstream iss(msg->data);
        std::vector<double> joint_angles;
        std::string token;

        while (std::getline(iss, token, ',')) {
            size_t eq_pos = token.find('=');
            if (eq_pos != std::string::npos) {
                double angle = std::stod(token.substr(eq_pos + 1));
                joint_angles.push_back(angle);
                RCLCPP_INFO(this->get_logger(), "Parsed angle: %f", angle);  // Log each parsed angle
            }
        }

        if (joint_angles.size() == 7) {
            std::vector<DHParameter> dh_params = {
                {0, M_PI, 0.0, 0.0},
                {joint_angles[0], M_PI_2, 0.0, -(0.1564 + 0.1284)},
                {joint_angles[1] + M_PI, M_PI_2, 0.0, -(0.0054 + 0.0064)},
                {joint_angles[2] + M_PI, M_PI_2, 0.0, -(0.2104 + 0.2104)},
                {joint_angles[3] + M_PI, M_PI_2, 0.0, -(0.0064 + 0.0064)},
                {joint_angles[4] + M_PI, M_PI_2, 0.0, -(0.2084 + 0.1059)},
                {joint_angles[5] + M_PI, M_PI_2, 0.0, 0.0},
                {joint_angles[6] + M_PI, M_PI, 0.0, -(0.1059 + 0.0615)}
            };

            Matrix4d T = forward_kinematics(dh_params);

            // Create a stringstream to format the matrix output
            std::ostringstream oss;
            oss << "Forward Kinematics Transformation Matrix:\n" << T;
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Expected 7 joint angles, but got %zu", joint_angles.size());
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForwardKinematicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
