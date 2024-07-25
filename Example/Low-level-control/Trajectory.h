//
// Created by 韩小龙 on 2024/7/2.
//

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <Eigen/Dense>

class Trajectory {
public:
    void rotationMatrixToFixedAngles(const Eigen::Matrix3d& R, float& alpha, float& beta, float& gamma);
    Eigen::Matrix<double, 1, 6> poly5_interpolation(float t0, float tf, float p0, float pf, float v0, float vf, float a0, float af);
    Eigen::MatrixXd TrajectoryGeneration(const Eigen::Matrix4d& TransMatrix, const Eigen::Matrix<double, 1, 6>& Final_pose, float t0, float T);
};

#endif //TRAJECTORY_H
