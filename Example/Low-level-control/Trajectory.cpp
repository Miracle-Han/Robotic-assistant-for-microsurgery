//
// Created by 韩小龙 on 2024/7/2.
//

#include "Trajectory.h"
#include "Jacobian.h"
#include "ForwardKinematic.h"
#include "InverseKinematic.h"
#include <iostream>
#include "Constants.h"


void Trajectory::rotationMatrixToFixedAngles(const Eigen::Matrix3d& R, float& alpha, float& beta, float& gamma) {
    beta = asin(-R(2,0));
    // 处理万向节锁定的情况
    if (std::abs(beta - M_PI / 2) < 1e-6) {
        // 当 beta = pi/2
        alpha = 0;
        gamma = atan2(R(0, 1), R(1, 1));
    } else if (std::abs(beta + M_PI / 2) < 1e-6) {
        // 当 beta = -pi/2
        alpha = 0;
        gamma = -atan2(R(0, 1), R(1, 1));
    } else {
        // 正常情况
        beta = atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
        alpha = atan2(R(1, 0), R(0, 0));
        gamma = atan2(R(2, 1), R(2, 2));
    }
}

Eigen::Matrix<double, 1, 6> Trajectory::poly5_interpolation(float t0, float tf, float p0, float pf, float v0, float vf, float a0, float af) {
    // 构建时间矩阵
    Eigen::Matrix<double, 6, 6> M;
    M << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5),
         0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4),
         0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3),
         1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5),
         0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4),
         0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3);

    // 构建边界条件向量
    Eigen::Matrix<double, 6, 1> b;
    b << p0, v0, a0, pf, vf, af;

    // 求解多项式系数
    Eigen::Matrix<double, 6, 1> a = M.colPivHouseholderQr().solve(b);

    // 返回多项式系数
    Eigen::Matrix<double, 1, 6> coeffs;
    coeffs << a(0), a(1), a(2), a(3), a(4), a(5);

    return coeffs;
}

Eigen::MatrixXd Trajectory::TrajectoryGeneration(const Eigen::Matrix4d& TransMatrix, const Eigen::Matrix<double, 1, 6>& Final_pose, float t0, float T) {
    Eigen::MatrixXd parameter_matrix(6, 6);

    // 提取旋转矩阵
    Eigen::Matrix3d rotation_matrix = TransMatrix.block<3, 3>(0, 0);

    // 计算固定角度
    float alpha_0, beta_0, gamma_0;
    rotationMatrixToFixedAngles(rotation_matrix, alpha_0, beta_0, gamma_0);

    // 提取位置部分
    float x_0 = static_cast<float>(TransMatrix(0, 3));
    float y_0 = static_cast<float>(TransMatrix(1, 3));
    float z_0 = static_cast<float>(TransMatrix(2, 3));

    // 定义初始位姿
    Eigen::Matrix<float, 1, 6> initial_pose;
    initial_pose << x_0, y_0, z_0, alpha_0, beta_0, gamma_0;

    // 进行插值计算
    for (int i = 0; i < 6; ++i) {
        float p0 = initial_pose(i);
        float pf = static_cast<float>(Final_pose(i));
        parameter_matrix.row(i) = poly5_interpolation(t0, T, p0, pf, 0, 0, 0, 0).cast<double>();
    }

    return parameter_matrix;
}