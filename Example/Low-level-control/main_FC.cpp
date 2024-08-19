/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#include "utilities.h"
#include "Jacobian.h"
#include "Constants.h"
#include "ForwardKinematic.h"
#include "InverseKinematic.h"
#include "Trajectory.h"
#include "SerialReader.h"

#include <Eigen/Dense>
#include <Windows.h>
#include <array>
#include <numeric>
#include <fstream>
#include <Eigen/Core>
#include <thread>
#include <mutex>


using namespace std;


#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>



namespace k_api = Kinova::Api;


#define PORT 10000
#define PORT_REAL_TIME 10001

#define DURATION 10             // Network timeout (seconds)


// Foot Control Setting
#define portname "COM11"
#define baudrate 460800
#define alpha 0.3       // for low-level filter
#define gamma 0.02     // Position Mapping coefficient

float velocity = 30.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)
double MicrosecondToSeconds = 1000000.0f;  // Unit conversion, microseconds to seconds
int Frequency = 400;           // Hz


std::mutex data_mutex;


// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)>
check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}

/*****************************
 * Example related function *
 *****************************/
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


/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    }
    else
    {
        bool action_finished = false;
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        {
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
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

/*********************************************
 * Thread 1: obtain Sensor Data From Adruino *
 ********************************************/
double x;
double y;
double z;
bool sensor_position_update = false;
bool LC0_ready = false;

void sensorDataThread() {
    // Arduino
    string port_name = portname; // 可以根据实际情况调整
    int baud_rate = baudrate;       // 可以根据实际情况调整
    // 创建串口读取对象
    SerialReader serialReader(port_name, baud_rate);

    // Unmixing Matrix
    constexpr std::array<double, 8> UM1 = {-0.2, 0.2, 0, 0, 0, 0, 0, 0};
    constexpr std::array<double, 8> UM2 = {0, 0, 0.15, -0.15, 0.15, -0.15, 0, 0};
    constexpr std::array<double, 8> UM3 = {0, 0, 0, 0, 0, 0, -0.2, 0.2};
    constexpr std::array<double, 8> UM4 = {0, -0.01, -0.3, 0.3, 0.25, -0.3, 0.08, -0.05};

    // Butterworth filter coefficient
    std::vector<double> b = {0.0730f,0.0730f};
    std::vector<double> a = {1.0f, -0.8541f};


    // 定义动态数组
    std::vector<int> array0, array1, array2, array3, array4, array5, array6, array7;

    std::array<double, 8> LC0;
    std::array<double, 8> LC;
    double x3_pre = 0.0;
    double y3_pre = 0.0;

    int64_t initial_time = GetTickUs();

    try
    {
        while (true) {
            vector<int> values = serialReader.readLineAsIntArray();
            int64_t current_t = GetTickUs();

            while ((current_t - initial_time)/MicrosecondToSeconds > 1.0 && (current_t - initial_time)/MicrosecondToSeconds < 3.0) {
                values = serialReader.readLineAsIntArray();

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
                double A0 = serialReader.calculateAverage(array0);
                double A1 = serialReader.calculateAverage(array1);
                double A2 = serialReader.calculateAverage(array2);
                double A3 = serialReader.calculateAverage(array3);
                double A4 = serialReader.calculateAverage(array4);
                double A5 = serialReader.calculateAverage(array5);
                double A6 = serialReader.calculateAverage(array6);
                double A7 = serialReader.calculateAverage(array7);

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

                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    LC0_ready = true;
                }

                break;
            }
        }

        std::vector<int> values = serialReader.readLineAsIntArray();

        // 初始化存储每个传感器的窗口
        std::vector<std::deque<double>> x_windows(values.size());
        std::vector<std::deque<double>> y_windows(values.size());

        for (int i = 0; i < values.size(); i++) {
            for (int j = 0; j < b.size(); j++ ) {
                x_windows[i].push_back(static_cast<double>(LC0[i]));
            }

            for (int j = 0; j < a.size(); j++ ) {
                y_windows[i].push_back(LC0[i]); // 初始化输出窗口
            }
        }


        while (true) {
            values = serialReader.readLineAsIntArray();

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
                LC[i] = y_filtered;
            }
            std::vector<double> LC_zscore(LC.size());
            for (size_t i = 0; i < LC.size(); ++i) {
                LC_zscore[i] = LC[i] - LC0[i];
            }

            double x33 = -std::inner_product(LC_zscore.begin(), LC_zscore.end(), UM2.begin(), 0.0);
            double y33 = std::inner_product(LC_zscore.begin(), LC_zscore.end(), UM1.begin(), 0.0);

            // 滤波计算
            double new_x = gamma * (x3_pre + (x33 - x3_pre) * alpha);
            double new_y = gamma * y33;
            double new_z = 0;

            // 判断当前 x 的值与前一次的值差别是否不超过 0.02
            if (std::abs(new_x - x3_pre) <= 0.02) {
                new_x = x3_pre;  // 如果差别不超过 0.02，当前值等于前一次的值
            }

            // 判断当前 x 的值与前一次的值差别是否不超过 0.02
            if (std::abs(new_y - y3_pre) <= 0.02) {
                new_y = y3_pre;  // 如果差别不超过 0.02，当前值等于前一次的值
            }

            {
                std::lock_guard<std::mutex> lock(data_mutex);
                y = new_y;
                z = new_z;
                sensor_position_update = true;
            }



            // 更新前一状态
            x3_pre = x;
            y3_pre = y;

        }
    }
    catch (std::exception& e)
    {
        cerr << "Error: " << e.what() << endl;
    }
}

/*********************************************
 * Thread 2: Calculate Position by New Pose *
 ********************************************/
bool moveingtohomeposition = false;
bool positionisupdate = false;
Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(7);


void PositionCalculateThread(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    while(true) {
        while(moveingtohomeposition) {
            // k_api::BaseCyclic::ActuatorFeedback actuator_feedback;
            k_api::BaseCyclic::Feedback base_feedback;
            k_api::BaseCyclic::Command  base_command;

            // Creat Class for Kinematic, Trajectory design, Jacobian Calculation
            ForwardKinematic fk;
            InverseKinematic ik;
            Trajectory traj;
            Jacobian jacobian;

            // Actuator data
            std::vector<float> commands = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            // Actuator data in radians
            std::vector<float> commands_rad = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

            auto servoingMode = k_api::Base::ServoingModeInformation();

            // Define Target pose
            double position_X;
            double position_Y;
            double position_Z;

            double orientationX;
            double orientationY;
            double orientationZ;

            //  Create a homogeneous transformation matrix for Target Pose
            // Target Pose: Matrix form
            Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity(); // 初始化为单位矩阵

            // Jacobian matrix
            Eigen::MatrixXd jacobian_matrix;
            Eigen::MatrixXd pseudo_inverse_jacobian_matrix;

            // Kinematic Calculation
            Eigen::Matrix4d T_Home_position;
            Eigen::Matrix4d current_FK;  // current homogeneous transformation matrix
            Eigen::Matrix3d Target_Rotation;

            // Actuator Incresement
            std::vector<float> solution;
            bool isConvergedAndWithLimit = true;

            try
            {
                // Set the base in low-level servoing mode
                servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
                base->SetServoingMode(servoingMode);
                base_feedback = base_cyclic->RefreshFeedback();

                int actuator_count = base->GetActuatorCount().count();

                while(moveingtohomeposition) {
                    // Actuator data
                    for(int i = 0; i < actuator_count; i++)
                    {
                        commands[i] = base_feedback.actuators(i).position();
                        std::cout << "Actuator"<< i << ": " << commands[i] << std::endl;
                    }
                    // degree to radian
                    for (int i = 0; i < actuator_count; i++) {
                        commands_rad[i] = commands[i] * M_PI / 180.0;
                    }

                    // Calculate forward kinematics
                    T_Home_position = fk.computeForwardKinematics(commands_rad);


                    bool local_LC0_ready = false;
                    // 进入实时控制
                    while (true) {

                        {
                            std::lock_guard<std::mutex> lock(data_mutex);
                            local_LC0_ready = LC0_ready;
                        }
                        double Local_position_x;
                        while (local_LC0_ready) {
                            // std::cout << "sensor_position_update: " << sensor_position_update << std::endl;
                            if(sensor_position_update) {
                                int j = 1;
                                int point_num = 1;

                                {
                                    std::lock_guard<std::mutex> lock(data_mutex);
                                    Local_position_x = y;
                                }
                                sensor_position_update = false;

                                while (j<point_num+1) {
                                    {
                                        // Define Target pose
                                        position_X = T_Home_position(0,3)+(Local_position_x*j*1/point_num);
                                        // +(Local_position_x*j*1/point_num)
                                        position_Y = T_Home_position(1,3);
                                        position_Z = T_Home_position(2,3);

                                        orientationX = M_PI/2.0f;
                                        orientationY = 0;
                                        orientationZ = M_PI/2.0f;

                                        // std::cout << "x position is : " << T_Home_position(0,3)+(Local_position_x*j*1/point_num) << std::endl;
                                    }


                                    // Position part
                                    target_pose(0,3) = position_X;
                                    target_pose(1,3) = position_Y;
                                    target_pose(2,3) = position_Z;

                                    std::cout << "target_pose_x = " <<  target_pose(0,3) << std::endl;

                                    // Orientation part
                                    Eigen::Matrix3d RotationMatrix = fk.computeRotationMatrix(orientationX,orientationY,orientationZ);
                                    target_pose.block<3,3>(0,0) = RotationMatrix;

                                    // Final Pose: vector form
                                    Eigen::Matrix<double, 1, 6> Final_pose;
                                    Final_pose << position_X, position_Y, position_Z, orientationX, orientationY, orientationZ;


                                    // 获取当前关节值用于计算delta q
                                    for(int i = 0; i < actuator_count; i++)
                                    {
                                        base_feedback = base_cyclic->RefreshFeedback();
                                        commands[i] = base_feedback.actuators(i).position();
                                        base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
                                        // std::cout << "Actuator"<< i << ": " << commands[i] << std::endl;
                                    }

                                    // degree to radian
                                    for (int i = 0; i < actuator_count; i++) {
                                        commands_rad[i] = commands[i] * M_PI / 180.0;
                                    }

                                    solution = commands_rad;


                                    Eigen::Matrix4d T_final = fk.computeForwardKinematics(commands_rad);
                                    // // Output the homogeneous transformation matrix of the forward kinematics
                                    // std::cout << "---------------------------------------------------------" << std::endl;
                                    std::cout << "Current X position is : " << T_final(0,3) << std::endl;
                                    // std::cout << T_final << std::endl;


                                    // Determine whether the Target pose has an inverse kinematics solution
                                    isConvergedAndWithLimit = ik.solveInverseKinematics(commands_rad, target_pose, 100, 1e-4).is_converged;
                                    if(isConvergedAndWithLimit) {
                                        solution = ik.solveInverseKinematics(commands_rad, target_pose, 100, 1e-4).joint_angles;
                                    }

                                    {
                                        std::lock_guard<std::mutex> lock(data_mutex);
                                        if(abs(target_pose(0,3)-T_final(0,3))>=0.001) {
                                            for (int i = 0; i < actuator_count; i++) {
                                                // Actuator data
                                                delta_q[i] = 10*((solution[i] - commands_rad[i]) * 180.0f / M_PI);
                                                std::cout << "delta_q "<< i << ": " << delta_q[i] << std::endl;
                                            }
                                        }else {
                                            for (int i = 0; i < actuator_count; i++) {
                                                // Actuator data
                                                delta_q[i] = 0.0f;
                                            }
                                        }

                                        positionisupdate = true;

                                        // std::cout <<"position is update" << std::endl;
                                    }
                                    j++;
                                }
                            }
                        }
                    }
                }
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex error: " << ex.what() << std::endl;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "Runtime error: " << ex2.what() << std::endl;
            }
        }
    }
}


/*********************************************
 * Thread 3: COntrol Mode *
 ********************************************/

void example_actuator_low_level_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    // Move arm to ready position
    example_move_to_home_position(base);

    // k_api::BaseCyclic::ActuatorFeedback actuator_feedback;
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    // Actuator data
    std::vector<float> commands = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // Actuator data in radians
    std::vector<float> commands_rad = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    auto servoingMode = k_api::Base::ServoingModeInformation();


    int64_t last = 0;
    int timeout = 0;

    {
        std::lock_guard<std::mutex> lock(data_mutex);
        moveingtohomeposition = true;
    }

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();


        int actuator_count = base->GetActuatorCount().count();

        // Actuator data
        for(int i = 0; i < actuator_count; i++)
        {
            commands[i] = base_feedback.actuators(i).position();
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
            std::cout << "Actuator"<< i << ": " << commands[i] << std::endl;
        }
        // degree to radian
        for (int i = 0; i < actuator_count; i++) {
            commands_rad[i] = commands[i] * M_PI / 180.0;
        }

        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            // std::cout << serialized_data << std::endl << std::endl;
        };

        Eigen::VectorXd local_delta_q = Eigen::VectorXd::Zero(7);
        int64_t temp = 0;

        // 进入实时控制
        while (true) {
            bool local_positionisupdate = false;
            Eigen::VectorXd error_q = Eigen::VectorXd::Zero(7);
            while (LC0_ready) {
                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    local_positionisupdate = positionisupdate;
                    positionisupdate = false;
                }

                if(local_positionisupdate) {
                    local_positionisupdate = false;
                    // local_delta_q += delta_q;
                    for(int i = 0; i < actuator_count; i++) {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        local_delta_q[i] = delta_q[i];

                        // error_q[i] = delta_q[i];
                        // std::cout << "local_delta_q "<< i << ": " << local_delta_q[i] << std::endl;
                    }
                }

                int64_t now = abs(GetTickUs());

                if(abs(now - temp) > 2000)
                {
                    temp = abs(now);
                    // std::cout << "test" << std::endl;
                    for(int i = 0; i < actuator_count; i++)
                    {
                        if(local_delta_q[i] >= 0.001f * velocity) {   // 0.001*20 = 0.02
                            commands[i] += (0.001f * velocity);
                            local_delta_q[i] = local_delta_q[i] - 0.001f * velocity;
                            base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                        }
                        else if(local_delta_q[i] <= -0.001f * velocity) {
                            commands[i] -= (0.001f * velocity);
                            local_delta_q[i] = local_delta_q[i] + 0.001f * velocity;
                            base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                        }
                        else {
                            // commands[i] = commands[i]+local_delta_q[i];
                            // local_delta_q[i] = 0.0f;
                        }

                        // base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                        // if(i == 0) {
                        //     std::cout << "local_delta_q " << i << ": " << local_delta_q[i] << std::endl;
                        //     std::cout << "commands " << i << ": " << commands[i] << std::endl;
                        // }
                    }
                    try
                    {
                        base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                    }
                    catch(...)
                    {
                        timeout++;
                    }
                    last = GetTickUs();
                }
            }
        }
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
    }
}

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);


    // 读取传感器数据线程
    std::thread ForceSensor_thread(sensorDataThread);
    // Controller
    std::thread control_thread(example_actuator_low_level_velocity_control, base, base_cyclic);
    // Delta_position calculate
    std::thread PositionCalculate_thread(PositionCalculateThread, base, base_cyclic);

    ForceSensor_thread.join();
    control_thread.join();
    PositionCalculate_thread.join();


    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}
