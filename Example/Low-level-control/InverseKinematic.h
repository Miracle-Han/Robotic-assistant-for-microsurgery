// InverseKinematic.h

#include <vector>
#include <Eigen/Dense>

struct IKResult {
    std::vector<float> joint_angles;
    bool is_converged;
};

class InverseKinematic {
public:
    Eigen::VectorXd computeError(const Eigen::Matrix4d& current_pose, const Eigen::Matrix4d& target_pose);
    IKResult solveInverseKinematics(const std::vector<float>& initial_joint_angles, const Eigen::Matrix4d& target_pose, int max_iterations, double tolerance);
};
