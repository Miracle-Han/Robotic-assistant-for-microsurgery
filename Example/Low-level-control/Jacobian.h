//
// Created by 韩小龙 on 24-7-2.
//

#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <vector>
#include <Eigen/Dense>

class jacobian {
public:
    static Eigen::Matrix4d computeJacobian(const std::vector<double>& joint_angles);
    static Eigen::MatrixXd computePseudoInverse(const Eigen::MatrixXd& Jacobian_matrix);
};

#endif //JACOBIAN_H
