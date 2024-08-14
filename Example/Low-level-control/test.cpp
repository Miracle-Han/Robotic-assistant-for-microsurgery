#include "SerialReader.h"
#include <iostream>
#include <Windows.h>
#include <vector>
#include <array>
#include <numeric>
#include <fstream>
#include <stdlib.h>
#include <deque>


#define portname "COM11"
#define baudrate 460800
#define alpha 0.3       // for low-level filter
int FrequencyofFC = 8333;  // 120hz

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

double butterworth_filter(const std::vector<double>& b, const std::vector<double>& a, std::deque<double>& x, std::deque<double>& y) {
    double output = b[0] * x.back();
    for (size_t i = 1; i < b.size(); ++i) {
        output += b[i] * x[x.size() - 1 - i];
    }
    for (size_t i = 1; i < a.size(); ++i) {
        output -= a[i] * y[y.size() - i];
    }
    return output;
}

int main() {
    string port_name = portname; // 可以根据实际情况调整
    int baud_rate = baudrate;       // 可以根据实际情况调整

    constexpr std::array<double, 8> UM1 = {-0.2, 0.2, 0, 0, 0, 0, 0, 0};
    constexpr std::array<double, 8> UM2 = {0, 0, 0.15, -0.15, 0.15, -0.15, 0, 0};
    constexpr std::array<double, 8> UM3 = {0, 0, 0, 0, 0, 0, -0.2, 0.2};
    constexpr std::array<double, 8> UM4 = {0, -0.01, -0.3, 0.3, 0.25, -0.3, 0.08, -0.05};

    double MicrosecondToSeconds = 1000000.0f;
    std::array<int, 8> LC0;
    std::array<int, 8> LC;
    double x3_pre = 0.0;
    double y3_pre = 0.0;



    int64_t initial_time = GetTickUs();

    // 定义动态数组
    std::vector<int> array0, array1, array2, array3, array4, array5, array6, array7;

    try {
        // 创建串口读取对象
        SerialReader serialReader(port_name, baud_rate);

        while (true) {
            vector<int> values = serialReader.readLineAsIntArray();

            int64_t current_t = GetTickUs();

            while ((current_t - initial_time)/MicrosecondToSeconds > 1.0 && (current_t - initial_time)/MicrosecondToSeconds < 3.0) {
                values = serialReader.readLineAsIntArray();
                if (!values.empty()) {
                    cout << "Received values: ";
                    for (int i = 0; i < values.size(); i++) {
                        cout << values[i] << " ";
                    }
                    cout << endl;
                }

                // 根据 values 更新每个动态数组
                if (values.size() >= 1) array0.push_back(values[0]);
                if (values.size() >= 2) array1.push_back(values[1]);
                if (values.size() >= 3) array2.push_back(values[2]);
                if (values.size() >= 4) array3.push_back(values[3]);
                if (values.size() >= 5) array4.push_back(values[4]);
                if (values.size() >= 6) array5.push_back(values[5]);
                if (values.size() >= 7) array6.push_back(values[6]);
                if (values.size() >= 8) array7.push_back(values[7]);

                current_t = GetTickUs();

            }
            if((current_t - initial_time)/MicrosecondToSeconds > 3.0) {
                int A0 = serialReader.calculateAverage(array0);
                int A1 = serialReader.calculateAverage(array1);
                int A2 = serialReader.calculateAverage(array2);
                int A3 = serialReader.calculateAverage(array3);
                int A4 = serialReader.calculateAverage(array4);
                int A5 = serialReader.calculateAverage(array5);
                int A6 = serialReader.calculateAverage(array6);
                int A7 = serialReader.calculateAverage(array7);

                // 输出平均值
                cout << "Averages:" << endl;
                cout << "A0: " << A0 << endl;
                cout << "A1: " << A1 << endl;
                cout << "A2: " << A2 << endl;
                cout << "A3: " << A3 << endl;
                cout << "A4: " << A4 << endl;
                cout << "A5: " << A5 << endl;
                cout << "A6: " << A6 << endl;
                cout << "A7: " << A7 << endl;

                LC0 = {A0, A1, A2, A3, A4, A5, A6, A7};
                break;
            }

        }

    } catch (std::exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    initial_time = GetTickUs();

    string filename = R"(F:\Imperial College London\FYP_Data\testforFC.txt)";
    ofstream dataFile;
    dataFile.open(filename);

    try {
        // 创建串口读取对象
        SerialReader serialReader(port_name, baud_rate);

        int iteration_num = 1;

        std::vector<double> b = {4.1655e-04f, 0.0012f, 0.0012f, 4.1655e-04f};
        std::vector<double> a = {1.0f, -2.68615f, 2.4197f, -0.7302f};

        int64_t temp = GetTickUs();
        int64_t now;


        vector<int> values = serialReader.readLineAsIntArray();
        values = serialReader.readLineAsIntArray();

        // 初始化存储每个传感器的窗口
        std::vector<std::deque<double>> x_windows(values.size());
        std::vector<std::deque<double>> y_windows(values.size());



        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < b.size(); j++ ) {
                x_windows[i].push_back(static_cast<double>(LC0[i]));
            }

            for (int j = 0; j < a.size(); j++ ) {
                y_windows[i].push_back(LC0[i]); // 初始化输出窗口
            }
        }

        while (true) {
            int64_t current_t = GetTickUs();

            now = GetTickUs();

            while ((current_t - initial_time) / MicrosecondToSeconds > 1.0) {
                values = serialReader.readLineAsIntArray();
                temp = now;
                now = GetTickUs();

                while (now - temp <= FrequencyofFC) {
                    now = GetTickUs();
                }

                current_t = GetTickUs();
                dataFile << "Total Running time for iteration " << iteration_num << " is: " << (current_t - initial_time) / MicrosecondToSeconds << endl;

                for (int i = 0; i < values.size(); i++) {

                    // 添加新值到输入信号窗口
                    x_windows[i].push_back(static_cast<double>(values[i]));
                    if (x_windows[i].size() >= b.size()) {
                        x_windows[i].pop_front(); // 保持窗口大小与滤波器阶数一致
                    }


                    // 应用巴特沃斯滤波器
                    double y_filtered = butterworth_filter(b, a, x_windows[i], y_windows[i]);


                    // 添加到输出信号窗口
                    y_windows[i].push_back(y_filtered);
                    if (y_windows[i].size() >= a.size()) {
                        y_windows[i].pop_front(); // 保持窗口大小与滤波器阶数一致
                    }

                    // std::cout << "Sensor " << i << " is " << values[i] << endl;
                    // std::cout << "Filtered Sensor " << i << " is " << y_filtered << endl;

                    // 写入原始和滤波后的数据到文件
                    dataFile << "Sensor " << i << " is " << values[i] << "; ";
                    dataFile << "Filtered Sensor " << i << " is " << y_filtered << "; ";
                    dataFile << endl;

                }

                iteration_num++;
            }
        }

    } catch (std::exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    return 0;
}










// #include <iostream>
// #include <fstream>
// #include <stdlib.h>
// using namespace std;
//
// int main()
// {
//     int a[2][3] = { 1, 2, 3, 4, 1,2};
//     float b = 11.1;
//
//     string filename = R"(F:\Imperial College London\FYP_Data\test.txt)";
//     // 向txt文档中写入数据
//     ofstream dataFile;
//
//     dataFile.open(filename);
//
//
//     dataFile << a[1][1] << ' '<< b << endl;     // 写入数据
//     dataFile.close();                           // 关闭文档
//
//     return 0;
// }
//













// #include <iostream>
// #include <vector>
// #include <Eigen/Dense>
// #include "Jacobian.h"
// #include "ForwardKinematic.h"
// #include "InverseKinematic.h"
// #include "Constants.h"
// #include "Trajectory.h"
// #include <chrono>
// #include <Windows.h>
//
// int64_t GetTickUs()
// {
// #if defined(_MSC_VER)
//     LARGE_INTEGER start, frequency;
//
//     QueryPerformanceFrequency(&frequency);
//     QueryPerformanceCounter(&start);
//
//     return (start.QuadPart * 1000000)/frequency.QuadPart;
// #else
//     struct timespec start;
//     clock_gettime(CLOCK_MONOTONIC, &start);
//
//     return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
// #endif
// }
//
//
// float M_pi = 3.1415926;
//
// // 将弧度转换为角度的函数
// float radToDeg(float rad) {
//     return rad * 180.0 / M_pi;
// }
//
// // 将角度转换为弧度的函数
// float degToRad(float deg) {
//     return deg * M_pi / 180.0;
// }
//
//
// int main(int argc, char **argv) {
//
//     // 定义并初始化一个std::vector来存储执行器的位置
//     // float q1 = 1.245226;
//     // float q2 = -1.014591;
//     // float q3 = -1.626709;
//     // float q4 = 1.306197;
//     // float q5 = -0.040284;
//     // float q6 = 0.686400;
//     // float q7 = -1.036441;
//     //
//     // std::vector<float> actuator_positions = {q1, q2, q3, q4, q5, q6, q7};
//
//
//
//     int64_t now = GetTickUs();
//     std::cout <<"time1 is " << now << std::endl;
//
//
//
//     // 关节角度定义
//     std::vector<float> actuator_positions = {359.999, 15.0062, 180.002, 230.001, 0.00146811, 55.0005, 89.99};
//
//     // 将角度值转换为弧度值
//     std::vector<float> actuator_positions_rad;
//     for (const auto& angle : actuator_positions) {
//         actuator_positions_rad.push_back(degToRad(angle));
//     }
//
//     std::cout << "actuator in degree: " << std::endl;
//     // 输出每个执行器的位置 - 角度值
//     for (size_t i = 0; i < actuator_positions.size(); ++i) {
//         std::cout << "actuator " << i << " position = " << actuator_positions[i] << std::endl;
//     }
//     std::cout << "---------------------------------------------------------" << std::endl;
//     std::cout << "actuator in radian: " << std::endl;
//     // 输出每个执行器的位置 - 弧度制
//     for (size_t i = 0; i < actuator_positions_rad.size(); ++i) {
//         std::cout << "actuator " << i << " position = " << actuator_positions_rad[i] << std::endl;
//     }
//
//     // 创建ForwardKinematics类的实例
//     ForwardKinematic fk;
//     // 计算正向运动学
//     Eigen::Matrix4d T_final = fk.computeForwardKinematics(actuator_positions_rad);
//     // 输出正向运动学的齐次变换矩阵
//     std::cout << "---------------------------------------------------------" << std::endl;
//     std::cout << "Final Homogeneous Transformation Matrix:" << std::endl;
//     std::cout << T_final << std::endl;
//
//
//     // 定义终止位姿
//     Eigen::Matrix<double, 1, 6> Final_pose;
//     Final_pose << 0.6, 0.4, 0.45, M_PI/2.0, M_PI/4.0, M_PI/2.0;
//
//     //
//     // // 轨迹生成
//     // Trajectory traj;
//     // float t0 =0.0f;
//     // float T = 10.0f;
//     // Eigen::MatrixXd para_matrix = traj.TrajectoryGeneration(T_final, Final_pose, t0, T);
//     //
//     // // 输出五项式插值参数
//     // std::cout << "---------------------------------------------------------" << std::endl;
//     // std::cout << "Five order interpoleration parameter: " << std::endl;
//     // std::cout << para_matrix << std::endl;
//     //
//     // std::cout << "---------------------------------------------------------" << std::endl;
//     // std::cout << "Five order interpoleration parameter of velocity: " << std::endl;
//     // Eigen::MatrixXd para_matrix_velocity = para_matrix.block<6,5>(0,1);
//     // std::cout << para_matrix_velocity << std::endl;
//
//
//
//
//     // // 创建Jacobian类的实例
//     // Jacobian jacobian;
//     // // 计算Jacobian矩阵
//     // Eigen::MatrixXd jacobian_matrix = jacobian.computeJacobian(actuator_positions_rad);
//     // std::cout << "---------------------------------------------------------" << std::endl;
//     // // 输出Jacobian矩阵
//     // std::cout << "Jacobian Matrix:" << std::endl;
//     // std::cout << jacobian_matrix << std::endl;
//     //
//     // std::cout << "---------------------------------------------------------" << std::endl;
//     // // 计算并输出Jacobian矩阵的伪逆
//     // Eigen::MatrixXd jacobian_pseudo_inverse = jacobian.computePseudoInverse(jacobian_matrix);
//     // std::cout << "Jacobian Pseudo-Inverse Matrix:" << std::endl;
//     // std::cout << jacobian_pseudo_inverse << std::endl;
//     //
//     //
//     //
//     //
//     std::cout << "---------------------------------------------------------" << std::endl;
//     // Inverse Kinematics
//     InverseKinematic ik;
//     Eigen::Matrix4d target_pose;
//     target_pose << 0, 0, 1, 0.6,
//                    1, 0, 0, -0.2,
//                    0, 0, 1, 0.6,
//                    0, 0, 0, 1;
//
//     // 求解逆运动学
//     std::vector<float> solution = ik.solveInverseKinematics(actuator_positions_rad, target_pose, 100, 1e-3).joint_angles;
//     //
//     // for (size_t i = 0; i < solution.size(); ++i) {
//     //     std::cout << "Inverse Kinemamtic solution: actuator " << i << " position in radian = " << solution[i] << std::endl;
//     // }
//
//     bool test = ik.solveInverseKinematics(actuator_positions_rad, target_pose, 100, 1e-3).is_converged;
//
//     // // 输出结果-弧度制
//     // std::cout << "Solution joint angles with radian:" << std::endl;
//     // for (const auto& angle : solution) {
//     //     std::cout << angle << " ";
//     // }
//     // std::cout << std::endl;
//     //
//     // // 输出结果-角度制
//     // std::cout << "---------------------------------------------------------" << std::endl;
//     // std::cout << "Solution joint angles in degree:" << std::endl;
//     // for (const auto& angle : solution) {
//     //     std::cout << radToDeg(angle) << " ";
//     // }
//     // std::cout << std::endl;
//
//     // 测试Inverse Kinematic result 是否正确
//     // 计算正向运动学
//     Eigen::Matrix4d T_final_ik = fk.computeForwardKinematics(solution);
//     // 输出正向运动学的齐次变换矩阵
//     std::cout << "---------------------------------------------------------" << std::endl;
//     std::cout << "Final Homogeneous Transformation Matrix for inverse Kinematic test:" << std::endl;
//     std::cout << T_final_ik << std::endl;
//
//
//     // // 获取初始时间点
//     // auto t_initial = std::chrono::high_resolution_clock::now();
//     //
//     // // 将初始时间点转换为double类型的时间值（以秒为单位）
//     // auto duration_since_epoch = t_initial.time_since_epoch();
//     // double t_initial_seconds = std::chrono::duration<double>(duration_since_epoch).count();
//     //
//     // double t_final = t_initial_seconds;
//     // double t_current = t_final;
//     // double t_0 = t_final - t_initial_seconds;
//     //
//     // // 输出结果
//     // std::cout << "Initial time point in seconds since epoch: " << t_initial_seconds << " seconds" << std::endl;
//
//     // auto t_initial = std::chrono::high_resolution_clock::now();
//     // auto t_final = t_initial;
//     // std::cout << "Initial time point in seconds since epoch: " << t_initial << " seconds" << std::endl;
//     //
//     //
//     // int64_t now2 = GetTickUs();
//     // std::cout <<"time2 is " << now2 << std::endl;
//     //
//     //
//     // double difference = (now2 - now)/1000000.0f;
//     // std::cout <<"difference is " << difference << std::endl;
//     // return 0;
// }
//
//
