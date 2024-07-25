#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <vector>
#include <Eigen/Dense>

class Jacobian {
public:
    Eigen::MatrixXd computeJacobian(const std::vector<float>& joint_angles);
    Eigen::MatrixXd computePseudoInverse(const Eigen::MatrixXd& J);
};

#endif // JACOBIAN_H
