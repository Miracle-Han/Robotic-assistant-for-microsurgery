//
// Created by 韩小龙 on 24-7-9.
//

#include "ForwardKinematic.h"
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>  // for std::cos and std::sin
#include "Constants.h"

using namespace std;

Eigen::Matrix4d transformationMatrix2(float alpha, float a, float d, float theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

Eigen::Matrix4d ForwardKinematic::computeForwardKinematics(const std::vector<float>& joint_angles) {
    if (joint_angles.size() != 7) {
        throw std::invalid_argument("Exactly 7 joint angles are required");
    }

    // Defining the DH matrix
    std::vector<std::vector<float>> DH_params = {
        {M_PI, 0.0, D0, 0.0},
        {M_PI / 2, 0.0, D1, joint_angles[0]},
        {M_PI / 2, 0.0, D2, joint_angles[1] + M_PI},
        {M_PI / 2, 0.0, D3, joint_angles[2] + M_PI},
        {M_PI / 2, 0.0, D4, joint_angles[3] + M_PI},
        {M_PI / 2, 0.0, D5, joint_angles[4] + M_PI},
        {M_PI / 2, 0.0, D6, joint_angles[5] + M_PI},
        {M_PI, 0.0, D7, joint_angles[6] + M_PI}
    };

    // Initialize the transformation matrix
    Eigen::Matrix4d T_final_handbook = Eigen::Matrix4d::Identity();

    // The transformation matrix of each joint is calculated and multiplied
    for (size_t i = 0; i < DH_params.size(); ++i) {
        float alpha = DH_params[i][0];
        float a = DH_params[i][1];
        float d = DH_params[i][2];
        float theta = DH_params[i][3];
        Eigen::Matrix4d T = transformationMatrix2(alpha, a, d, theta);
        T_final_handbook = T_final_handbook * T;
    }

    return T_final_handbook;
}


Eigen::Matrix3d ForwardKinematic::computeRotationMatrix(const double & orientationX,const double & orientationY, const double & orientationZ) {

    Eigen::Matrix3d RotationMatrix;
    Eigen::Matrix3d RotationOfZ;
    Eigen::Matrix3d RotationOfY;
    Eigen::Matrix3d RotationOfX;


    // Fixed angle method: XYZ
    // Calculate the rotation matrix about the Z axis
    RotationOfZ <<
        cos(orientationZ), -sin(orientationZ), 0,
        sin(orientationZ),  cos(orientationZ), 0,
        0,                 0,                 1;

    // Calculate the rotation matrix about the Y axis
    RotationOfY <<
        cos(orientationY), 0, sin(orientationY),
        0,                1, 0,
        -sin(orientationY), 0, cos(orientationY);

    // Calculate the rotation matrix about the X axis
    RotationOfX <<
        1, 0,                 0,
        0, cos(orientationX), -sin(orientationX),
        0, sin(orientationX),  cos(orientationX);

    RotationMatrix = RotationOfZ*RotationOfY*RotationOfX;
    return RotationMatrix;
}


