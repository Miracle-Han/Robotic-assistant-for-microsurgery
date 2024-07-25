#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Jacobian.h"
#include "ForwardKinematic.h"
#include "InverseKinematic.h"
#include "Constants.h"
#include "Trajectory.h"
#include <chrono>
#include <Windows.h>

int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000)/frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}


float M_pi = 3.1415926;

// 将弧度转换为角度的函数
float radToDeg(float rad) {
    return rad * 180.0 / M_pi;
}

// 将角度转换为弧度的函数
float degToRad(float deg) {
    return deg * M_pi / 180.0;
}


int main(int argc, char **argv) {

    // 定义并初始化一个std::vector来存储执行器的位置
    // float q1 = 1.245226;
    // float q2 = -1.014591;
    // float q3 = -1.626709;
    // float q4 = 1.306197;
    // float q5 = -0.040284;
    // float q6 = 0.686400;
    // float q7 = -1.036441;
    //
    // std::vector<float> actuator_positions = {q1, q2, q3, q4, q5, q6, q7};



    int64_t now = GetTickUs();
    std::cout <<"time1 is " << now << std::endl;




    // 关节角度定义
    std::vector<float> actuator_positions = {359.999, 15.0062, 180.002, 230.001, 0.00146811, 55.0005, 89.99};

    // 将角度值转换为弧度值
    std::vector<float> actuator_positions_rad;
    for (const auto& angle : actuator_positions) {
        actuator_positions_rad.push_back(degToRad(angle));
    }

    std::cout << "actuator in degree: " << std::endl;
    // 输出每个执行器的位置 - 角度值
    for (size_t i = 0; i < actuator_positions.size(); ++i) {
        std::cout << "actuator " << i << " position = " << actuator_positions[i] << std::endl;
    }
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "actuator in radian: " << std::endl;
    // 输出每个执行器的位置 - 弧度制
    for (size_t i = 0; i < actuator_positions_rad.size(); ++i) {
        std::cout << "actuator " << i << " position = " << actuator_positions_rad[i] << std::endl;
    }

    // 创建ForwardKinematics类的实例
    ForwardKinematic fk;
    // 计算正向运动学
    Eigen::Matrix4d T_final = fk.computeForwardKinematics(actuator_positions_rad);
    // 输出正向运动学的齐次变换矩阵
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "Final Homogeneous Transformation Matrix:" << std::endl;
    std::cout << T_final << std::endl;


    // 定义终止位姿
    Eigen::Matrix<double, 1, 6> Final_pose;
    Final_pose << 0.6, 0.4, 0.45, M_PI/2.0, M_PI/4.0, M_PI/2.0;

    //
    // // 轨迹生成
    // Trajectory traj;
    // float t0 =0.0f;
    // float T = 10.0f;
    // Eigen::MatrixXd para_matrix = traj.TrajectoryGeneration(T_final, Final_pose, t0, T);
    //
    // // 输出五项式插值参数
    // std::cout << "---------------------------------------------------------" << std::endl;
    // std::cout << "Five order interpoleration parameter: " << std::endl;
    // std::cout << para_matrix << std::endl;
    //
    // std::cout << "---------------------------------------------------------" << std::endl;
    // std::cout << "Five order interpoleration parameter of velocity: " << std::endl;
    // Eigen::MatrixXd para_matrix_velocity = para_matrix.block<6,5>(0,1);
    // std::cout << para_matrix_velocity << std::endl;




    // // 创建Jacobian类的实例
    // Jacobian jacobian;
    // // 计算Jacobian矩阵
    // Eigen::MatrixXd jacobian_matrix = jacobian.computeJacobian(actuator_positions_rad);
    // std::cout << "---------------------------------------------------------" << std::endl;
    // // 输出Jacobian矩阵
    // std::cout << "Jacobian Matrix:" << std::endl;
    // std::cout << jacobian_matrix << std::endl;
    //
    // std::cout << "---------------------------------------------------------" << std::endl;
    // // 计算并输出Jacobian矩阵的伪逆
    // Eigen::MatrixXd jacobian_pseudo_inverse = jacobian.computePseudoInverse(jacobian_matrix);
    // std::cout << "Jacobian Pseudo-Inverse Matrix:" << std::endl;
    // std::cout << jacobian_pseudo_inverse << std::endl;
    //
    //
    //
    //
    std::cout << "---------------------------------------------------------" << std::endl;
    // Inverse Kinematics
    InverseKinematic ik;
    Eigen::Matrix4d target_pose;
    target_pose << 0, 0, 1, 0.6,
                   1, 0, 0, -0.2,
                   0, 0, 1, 0.6,
                   0, 0, 0, 1;

    // 求解逆运动学
    std::vector<float> solution = ik.solveInverseKinematics(actuator_positions_rad, target_pose, 100, 1e-3).joint_angles;
    //
    // for (size_t i = 0; i < solution.size(); ++i) {
    //     std::cout << "Inverse Kinemamtic solution: actuator " << i << " position in radian = " << solution[i] << std::endl;
    // }

    bool test = ik.solveInverseKinematics(actuator_positions_rad, target_pose, 100, 1e-3).is_converged;

    // // 输出结果-弧度制
    // std::cout << "Solution joint angles with radian:" << std::endl;
    // for (const auto& angle : solution) {
    //     std::cout << angle << " ";
    // }
    // std::cout << std::endl;
    //
    // // 输出结果-角度制
    // std::cout << "---------------------------------------------------------" << std::endl;
    // std::cout << "Solution joint angles in degree:" << std::endl;
    // for (const auto& angle : solution) {
    //     std::cout << radToDeg(angle) << " ";
    // }
    // std::cout << std::endl;

    // 测试Inverse Kinematic result 是否正确
    // 计算正向运动学
    Eigen::Matrix4d T_final_ik = fk.computeForwardKinematics(solution);
    // 输出正向运动学的齐次变换矩阵
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "Final Homogeneous Transformation Matrix for inverse Kinematic test:" << std::endl;
    std::cout << T_final_ik << std::endl;


    // // 获取初始时间点
    // auto t_initial = std::chrono::high_resolution_clock::now();
    //
    // // 将初始时间点转换为double类型的时间值（以秒为单位）
    // auto duration_since_epoch = t_initial.time_since_epoch();
    // double t_initial_seconds = std::chrono::duration<double>(duration_since_epoch).count();
    //
    // double t_final = t_initial_seconds;
    // double t_current = t_final;
    // double t_0 = t_final - t_initial_seconds;
    //
    // // 输出结果
    // std::cout << "Initial time point in seconds since epoch: " << t_initial_seconds << " seconds" << std::endl;

    // auto t_initial = std::chrono::high_resolution_clock::now();
    // auto t_final = t_initial;
    // std::cout << "Initial time point in seconds since epoch: " << t_initial << " seconds" << std::endl;
    //
    //
    // int64_t now2 = GetTickUs();
    // std::cout <<"time2 is " << now2 << std::endl;
    //
    //
    // double difference = (now2 - now)/1000000.0f;
    // std::cout <<"difference is " << difference << std::endl;
    // return 0;
}


