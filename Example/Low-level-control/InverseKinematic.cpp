// InverseKinematic.cpp

#include "InverseKinematic.h"
#include "Jacobian.h"
#include <iostream>

#include "Constants.h"
#include "ForwardKinematic.h"


// 定义关节位置限制
const std::vector<std::pair<float, float>> joint_limits = {
    {-INFINITY, INFINITY}, // Actuator 1
    {-128.9f, 128.9f},     // Actuator 2
    {-INFINITY, INFINITY}, // Actuator 3
    {-147.8f, 147.8f},     // Actuator 4
    {-INFINITY, INFINITY}, // Actuator 5
    {-120.3f, 120.3f},     // Actuator 6
    {-INFINITY, INFINITY}  // Actuator 7
};

Eigen::VectorXd InverseKinematic::computeError(const Eigen::Matrix4d& current_pose, const Eigen::Matrix4d& target_pose) {
    // 计算位置误差
    Eigen::Vector3d current_position = current_pose.block<3, 1>(0, 3);
    Eigen::Vector3d target_position = target_pose.block<3, 1>(0, 3);
    Eigen::Vector3d position_error = target_position - current_position;

    // 计算旋转误差（使用旋转矩阵）
    Eigen::Matrix3d current_rotation = current_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d target_rotation = target_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d rotation_error_matrix = target_rotation * current_rotation.transpose();
    Eigen::AngleAxisd rotation_error_angle_axis(rotation_error_matrix);
    Eigen::Vector3d rotation_error = rotation_error_angle_axis.angle() * rotation_error_angle_axis.axis();

    // 组合位置和旋转误差
    Eigen::VectorXd error(6);
    error.head<3>() = position_error;
    error.tail<3>() = rotation_error;

    return error;
}

IKResult InverseKinematic::solveInverseKinematics(const std::vector<float>& initial_joint_angles, const Eigen::Matrix4d& target_pose, int max_iterations, double tolerance) {

    ForwardKinematic fk;
    Jacobian jacobian;

    std::vector<float> joint_angles = initial_joint_angles;

    // 检查并调整第 2 个元素（索引 1）
    if (joint_angles[1]*180/M_PI > 180.0f) {
        joint_angles[1] -= 2*M_PI;
    }

    // 检查并调整第 4 个元素（索引 3）
    if (joint_angles[3]*180/M_PI > 180.0f) {
        joint_angles[3] -= 2*M_PI;
    }

    // 检查并调整第 6 个元素（索引 5）
    if (joint_angles[5]*180/M_PI > 180.0f) {
        joint_angles[5] -= 2*M_PI;
    }


    bool is_converged = false;
    bool is_within_limits = true;

    for (int i = 0; i < max_iterations; ++i) {
        Eigen::Matrix4d current_pose = fk.computeForwardKinematics(joint_angles);
        Eigen::VectorXd error = computeError(current_pose, target_pose);

        // 计算雅可比矩阵
        Eigen::MatrixXd J = jacobian.computeJacobian(joint_angles);

        // 计算雅可比矩阵的伪逆
        Eigen::MatrixXd J_pseudo_inverse = jacobian.computePseudoInverse(J);

        // 计算关节角度的更新量
        Eigen::VectorXd delta_theta = J_pseudo_inverse * error;

        // 更新关节角度并应用限制
        for (size_t j = 0; j < joint_angles.size(); ++j) {
            joint_angles[j] += delta_theta(j);
        }

        // 检查误差是否在容忍范围内
        if (error.norm() < tolerance) {
            std::cout << "Converged in " << i + 1 << " iterations." << std::endl;
            is_converged = true;
            break;
        }

    }

    // 检查并应用关节位置限制/
    for (size_t j = 0; j < joint_angles.size(); ++j) {
        if ((joint_angles[j]* 180.0f/M_PI) < joint_limits[j].first || (joint_angles[j]* 180.0f/M_PI) > joint_limits[j].second) {
            std::cout << "Joint " << j+1 << " exceeded its limits." << std::endl;
            is_within_limits = false;
        }
    }

    if (!is_converged) {
        std::cout << "Did not converge within the maximum number of iterations." << std::endl;
    }

    return {joint_angles, is_converged && is_within_limits};
}
