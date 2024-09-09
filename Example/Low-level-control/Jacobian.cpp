#include "Jacobian.h"
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <cmath>  // for std::cos and std::sin
#include "Constants.h"

using namespace std;


Eigen::Matrix4d transformationMatrix(float alpha, float a, float d, float theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

Eigen::MatrixXd Jacobian::computeJacobian(const std::vector<float>& joint_angles) {

    Eigen::MatrixXd Jacobian_matrix(6, 7);  // Update to 6x7 matrix

    if (joint_angles.size() != 7) {
        throw std::invalid_argument("Exactly 7 joint angles are required");
    } else {
        float joint_values[7];
        for (size_t i = 0; i < 7; ++i) {
            joint_values[i] = joint_angles[i];
        }

        // Define DH Matrix
        std::vector<std::vector<float>> DH_params = {
            {M_PI / 2, 0.0, D1, joint_values[0]},
            {M_PI / 2, 0.0, D2, joint_values[1] + M_PI},
            {M_PI / 2, 0.0, D3, joint_values[2] + M_PI},
            {M_PI / 2, 0.0, D4, joint_values[3] + M_PI},
            {M_PI / 2, 0.0, D5, joint_values[4] + M_PI},
            {M_PI / 2, 0.0, D6, joint_values[5] + M_PI},
            {M_PI, 0.0, D7, joint_values[6] + M_PI}
        };

        // Initialize the transformation matrix
        Eigen::Matrix4d T0_n = Eigen::Matrix4d::Identity();
        T0_n(1, 1) = -1;
        T0_n(2, 2) = -1;

        int num_joints = 7;

        // Initializes the position and Z-axis vector
        Eigen::MatrixXd positions(3, num_joints + 1);
        Eigen::MatrixXd z_vectors(3, num_joints + 1);
        positions.setZero();
        z_vectors.setZero();
        z_vectors.col(0) = T0_n.block<3, 1>(0, 2);  // 基坐标系的Z轴

        // Calculate the position and z-axis vector
        for (int i = 0; i < num_joints; ++i) {
            float alpha = DH_params[i][0];
            float a = DH_params[i][1];
            float d = DH_params[i][2];
            float theta = DH_params[i][3];

            Eigen::Matrix4d T_i = transformationMatrix(alpha, a, d, theta);
            T0_n = T0_n * T_i;

            positions.col(i + 1) = T0_n.block<3, 1>(0, 3);
            z_vectors.col(i + 1) = T0_n.block<3, 1>(0, 2);
        }

        // Computes Jacobian matrices
        for (int i = 0; i < num_joints; ++i) {
            float cross_x = z_vectors(1, i) * (positions(2, num_joints) - positions(2, i)) - z_vectors(2, i) * (positions(1, num_joints) - positions(1, i));
            float cross_y = z_vectors(2, i) * (positions(0, num_joints) - positions(0, i)) - z_vectors(0, i) * (positions(2, num_joints) - positions(2, i));
            float cross_z = z_vectors(0, i) * (positions(1, num_joints) - positions(1, i)) - z_vectors(1, i) * (positions(0, num_joints) - positions(0, i));

            Jacobian_matrix(0, i) = cross_x;
            Jacobian_matrix(1, i) = cross_y;
            Jacobian_matrix(2, i) = cross_z;

            Jacobian_matrix(3, i) = z_vectors(0, i);
            Jacobian_matrix(4, i) = z_vectors(1, i);
            Jacobian_matrix(5, i) = z_vectors(2, i);
        }
    }
    return Jacobian_matrix;
}

Eigen::MatrixXd Jacobian::computePseudoInverse(const Eigen::MatrixXd& J) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = std::numeric_limits<double>::epsilon() * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff();

    Eigen::VectorXd singularValuesInv = svd.singularValues();
    for (Eigen::Index i = 0; i < singularValuesInv.size(); ++i) {
        if (singularValuesInv(i) > tolerance) {
            singularValuesInv(i) = 1.0 / singularValuesInv(i);
        } else {
            singularValuesInv(i) = 0;
        }
    }
    return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().adjoint();
}