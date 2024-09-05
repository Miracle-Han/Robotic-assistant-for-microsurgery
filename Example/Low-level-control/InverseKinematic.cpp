// InverseKinematic.cpp

#include "InverseKinematic.h"
#include "Jacobian.h"
#include <iostream>

#include "Constants.h"
#include "ForwardKinematic.h"


// Define joint angle limits  (degree)
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
    // Calculated position error
    Eigen::Vector3d current_position = current_pose.block<3, 1>(0, 3);
    Eigen::Vector3d target_position = target_pose.block<3, 1>(0, 3);
    Eigen::Vector3d position_error = target_position - current_position;

    // Calculate rotation error (using rotation matrix)
    Eigen::Matrix3d current_rotation = current_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d target_rotation = target_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d rotation_error_matrix = target_rotation * current_rotation.transpose();
    Eigen::AngleAxisd rotation_error_angle_axis(rotation_error_matrix);
    Eigen::Vector3d rotation_error = rotation_error_angle_axis.angle() * rotation_error_angle_axis.axis();

    // Combined position and rotation error
    Eigen::VectorXd error(6);
    error.head<3>() = position_error;
    error.tail<3>() = rotation_error;

    return error;
}

IKResult InverseKinematic::solveInverseKinematics(const std::vector<float>& initial_joint_angles, const Eigen::Matrix4d& target_pose, int max_iterations, double tolerance) {

    ForwardKinematic fk;
    Jacobian jacobian;

    std::vector<float> joint_angles = initial_joint_angles;

    // Check joint angle limition
    // Check Joint 2
    if (joint_angles[1]*180/M_PI > 180.0f) {
        joint_angles[1] -= 2*M_PI;
    }

    // heck Joint 4
    if (joint_angles[3]*180/M_PI > 180.0f) {
        joint_angles[3] -= 2*M_PI;
    }

    // heck Joint 6
    if (joint_angles[5]*180/M_PI > 180.0f) {
        joint_angles[5] -= 2*M_PI;
    }


    bool is_converged = false;
    bool is_within_limits = true;

    for (int i = 0; i < max_iterations; ++i) {
        Eigen::Matrix4d current_pose = fk.computeForwardKinematics(joint_angles);
        Eigen::VectorXd error = computeError(current_pose, target_pose);

        // Calculate the Jacobian matrix
        Eigen::MatrixXd J = jacobian.computeJacobian(joint_angles);

        // Calculate the Pseudo-Inverse Jacobian matrix
        Eigen::MatrixXd J_pseudo_inverse = jacobian.computePseudoInverse(J);

        // The amount of increasement of joint Angle is calculated
        Eigen::VectorXd delta_theta = J_pseudo_inverse * error;

        // Update joint angles and apply limits
        for (size_t j = 0; j < joint_angles.size(); ++j) {
            joint_angles[j] += delta_theta(j);
        }

        // Check whether the error is within tolerance
        if (error.norm() < tolerance) {
            // std::cout << "Converged in " << i + 1 << " iterations." << std::endl;
            is_converged = true;
            break;
        }
    }

    // Check and apply joint position restrictions
    for (size_t j = 0; j < joint_angles.size(); ++j) {
        if ((joint_angles[j]* 180.0f/M_PI) < joint_limits[j].first || (joint_angles[j]* 180.0f/M_PI) > joint_limits[j].second) {
            std::cout << "Joint " << j+1 << " exceeded its limits." << std::endl;
            is_within_limits = false;
        }
    }

    if (!is_converged) {
        std::cout << "Did not converge within the maximum number of iterations." << std::endl;
    }

    if (is_converged && is_within_limits) {
        // Check Joint 2
        if (joint_angles[1] < 0.0f) {
            joint_angles[1] += 2*M_PI;
        }

        // Check Joint 4
        if (joint_angles[3] < 0.0f) {
            joint_angles[3] += 2*M_PI;
        }

        // Check Joint 6
        if (joint_angles[5] < 0.0f) {
            joint_angles[5] += 2*M_PI;
        }
    }

    return {joint_angles, is_converged && is_within_limits};
}
