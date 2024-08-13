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
#include<Eigen/Core>


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

float velocity = 40.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)
double MicrosecondToSeconds = 1000000.0f;  // Unit conversion, microseconds to seconds
int Frequency = 1000;           // Hz

int FrequencyofFC = 8333;  // 120hz


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



bool example_actuator_low_level_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool return_status = true;

    // Move arm to ready position
    example_move_to_home_position(base);

    // k_api::BaseCyclic::ActuatorFeedback actuator_feedback;
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    auto servoingMode = k_api::Base::ServoingModeInformation();



    // Actuator data in radians
    std::vector<float> commands_rad = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};



    int64_t last = 0;


    int timeout = 0;

    // Arduino
    string port_name = portname; // 可以根据实际情况调整
    int baud_rate = baudrate;       // 可以根据实际情况调整

    // Unmixing Matrix
    constexpr std::array<double, 8> UM1 = {-0.2, 0.2, 0, 0, 0, 0, 0, 0};
    constexpr std::array<double, 8> UM2 = {0, 0, 0.15, -0.15, 0.15, -0.15, 0, 0};
    constexpr std::array<double, 8> UM3 = {0, 0, 0, 0, 0, 0, -0.2, 0.2};
    constexpr std::array<double, 8> UM4 = {0, -0.01, -0.3, 0.3, 0.25, -0.3, 0.08, -0.05};

    //
    std::array<int, 8> LC0;
    std::array<int, 8> LC;
    double x3_pre = 0.0;
    double y3_pre = 0.0;

    // 定义动态数组
    std::vector<int> array0, array1, array2, array3, array4, array5, array6, array7;

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
    Eigen::Matrix4d current_FK;  // current homogeneous transformation matrix
    Eigen::VectorXd current_pose_interploration(6); // Expect position of Trajectory
    Eigen::Matrix3d current_Rotation;
    Eigen::Matrix3d Target_Rotation;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try
    {
        // Creat Class for Kinematic, Trajectory design, Jacobian Calculation
        ForwardKinematic fk;
        InverseKinematic ik;
        Trajectory traj;
        Jacobian jacobian;

        // 创建串口读取对象
        SerialReader serialReader(port_name, baud_rate);

        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();



        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            // std::cout << serialized_data << std::endl << std::endl;
        };

        int64_t initial_time = GetTickUs();

        // Get message from Arduino
        while (true) {
            vector<int> values = serialReader.readLineAsIntArray();
            int64_t current_t = GetTickUs();

            while ((current_t - initial_time)/MicrosecondToSeconds > 1.0 && (current_t - initial_time)/MicrosecondToSeconds < 3.0) {
                values = serialReader.readLineAsIntArray();
                if (!values.empty()) {
                    // cout << "Received values: ";
                    // for (int i = 0; i < values.size(); i++) {
                    //     cout << values[i] << " ";
                    // }
                    // cout << endl;
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

        initial_time = GetTickUs();


        int actuator_count = base->GetActuatorCount().count();
        // Actuator data
        std::vector<float> commands;
        std::cout << "=====================================" << std::endl;
        for(int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
            std::cout << "Actuator"<< i << ": " << commands[i] << std::endl;
        }
        // degree to radian
        for (int i = 0; i < actuator_count; i++) {
            commands_rad[i] = commands[i] * M_PI / 180.0;
        }

        // Calculate forward kinematics
        Eigen::Matrix4d T_Home_position = fk.computeForwardKinematics(commands_rad);

        vector<int> values = serialReader.readLineAsIntArray();


        while (true) {
            int64_t current_t = GetTickUs();
            values = serialReader.readLineAsIntArray();

            while ((current_t - initial_time)/MicrosecondToSeconds > 1.0) {
                values = serialReader.readLineAsIntArray();

                for (int i = 0; i < values.size(); ++i) {
                    LC[i] = values[i];
                }

                std::vector<double> LC_zscore(LC.size());
                for (size_t i = 0; i < LC.size(); ++i) {
                    LC_zscore[i] = LC[i] - LC0[i];
                }

                double x33 = -std::inner_product(LC_zscore.begin(), LC_zscore.end(), UM2.begin(), 0.0);
                double y33 = std::inner_product(LC_zscore.begin(), LC_zscore.end(), UM1.begin(), 0.0);

                // 滤波计算
                double x = x3_pre + (x33 - x3_pre) * alpha;
                double y = y3_pre + (y33 - y3_pre) * alpha;


                // 更新前一状态
                x3_pre = x;
                y3_pre = y;


                // Define Target pose
                position_X = T_Home_position(0,3) + 0.01*y;
                position_Y = T_Home_position(1,3);
                position_Z = T_Home_position(2,3);

                std::cout << "x position is ================= " << 0.1*x << std::endl;
                std::cout << "x position is ================= " << position_X << std::endl;
                std::cout << "current time is " << GetTickUs << std::endl;

                orientationX = M_PI/2.0f;
                orientationY = 0;
                orientationZ = M_PI/2.0f;

                // Position part
                target_pose(0,3) = position_X;
                target_pose(1,3) = position_Y;
                target_pose(2,3) = position_Z;

                // Orientation part
                Eigen::Matrix3d RotationMatrix = fk.computeRotationMatrix(orientationX,orientationY,orientationZ);
                target_pose.block<3,3>(0,0) = RotationMatrix;


                // Final Pose: vector form
                Eigen::Matrix<double, 1, 6> Final_pose;
                Final_pose << position_X, position_Y, position_Z, orientationX, orientationY, orientationZ;

                // Determine whether the Target pose has an inverse kinematics solution
                bool isConvergedAndWithLimit = ik.solveInverseKinematics(commands_rad, target_pose, 100, 1e-3).is_converged;
                std::vector<float> solution = ik.solveInverseKinematics(commands_rad, target_pose, 100, 1e-3).joint_angles;


                // Actuator Incresement
                Eigen::VectorXd delta_q(7);

                for (int i = 0; i < actuator_count; i++) {
                    delta_q[i] = (solution[i] - commands_rad[i]) * 180.0f / M_PI;

                    commands_rad[i] = solution[i];
                }

                last = 0;

                while (isConvergedAndWithLimit == true && delta_q.cwiseAbs().sum() > 10e-3) {
                    // std::cout << "test1111111111111" << std::endl;
                    int64_t now = GetTickUs();
                    if(last == 0) {
                        now = abs(now);
                    }
                    if(now - last > 1000)
                    {
                        for(int i = 0; i < actuator_count; i++)
                        {
                            if(delta_q[i] >= 0.001f * velocity) {   // 0.001*20 = 0.02
                                commands[i] += (0.001f * velocity);
                                delta_q[i] = delta_q[i] - 0.001f * velocity;
                                base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                            }
                            else if(delta_q[i] <= -0.001f * velocity) {
                                commands[i] -= (0.001f * velocity);
                                delta_q[i] = delta_q[i] + 0.001f * velocity;
                                base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                            }
                            else {
                                commands[i] = commands[i]+delta_q[i];
                                delta_q[i] = 0.0f;
                            }

                            // std::cout << commands[i] <<std::endl;
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

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }
    catch (std::exception& e)
    {
        cerr << "Error: " << e.what() << endl;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
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

    // Example core
    auto isOk = example_actuator_low_level_velocity_control(base, base_cyclic);
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in example_cyclic_armbase() function." << std::endl;
    }

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
