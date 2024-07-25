//
// Created by 韩小龙 on 24-7-9.
//

#ifndef FORWARDKINEMATIC_H
#define FORWARDKINEMATIC_H


#include <vector>
#include <Eigen/Dense>

class ForwardKinematic {
public:
    Eigen::Matrix4d computeForwardKinematics(const std::vector<float>& joint_angles);
    Eigen::Matrix3d ForwardKinematic::computeRotationMatrix(const double & orientationX,const double & orientationY, const double & orientationZ);
};

#endif // FORWARDKINEMATICS_H