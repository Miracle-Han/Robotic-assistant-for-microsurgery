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
#include <Eigen/Dense>
#include "Constants.h"
#include "ForwardKinematic.h"
#include "InverseKinematic.h"
#include "Trajectory.h"

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>



namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

#define DURATION 5             // Network timeout (seconds)

float velocity = 20.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)

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

// // 将弧度转换为角度的函数
// float radToDeg(float rad)
// {
//     return rad * 180.0 / M_PI;
// }
//
// // 将角度转换为弧度的函数
// float degToRad(float deg)
// {
//     return deg;
// }


bool example_actuator_low_level_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool return_status = true;

    // Move arm to ready position
    example_move_to_home_position(base);



    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    // k_api::BaseCyclic::ActuatorFeedback actuator_feedback;

    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;



    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try
    {
        // Creat Class for Kinematic, Trajectory design, Jacobian Calculation
        ForwardKinematic fk;
        InverseKinematic ik;
        Trajectory traj;
        Jacobian jacobian;

        // Unit conversion, microseconds to seconds
        double MicrosecondToSeconds = 1000000.0f;

        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for(int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());

            // 读取每个actuator的位置信息-Degree
            std::cout << "actuator " << i+1 << " position = " << commands[i] << std::endl;

        }

        // degree to radian
        std::vector<float> commands_rad;
        for (const auto& angle: commands) {
            commands_rad.push_back(angle * M_PI / 180.0);
        }
        std::cout << "actuator in radian: " << std::endl;
        for (size_t i = 0; i < commands_rad.size(); ++i) {
            std::cout << "actuator " << i+1 << " position in radian = " << commands_rad[i] << std::endl;
        }


        // Calculate and output forward kinematics results
        Eigen::Matrix4d T_Home_position = fk.computeForwardKinematics(commands_rad);
        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "Initial(home position) Homogeneous Transformation Matrix:" << std::endl;
        std::cout << T_Home_position  << std::endl;


        // Define Target pose
        double position_X = 0.6f;
        double position_Y = -0.2f;
        double position_Z = 0.6f;

        double orientationX = M_PI/2.0f;
        double orientationY = M_PI/3.0f;
        double orientationZ = M_PI/3.0f;


        //  Create a homogeneous transformation matrix for Target Pose
        // Target Pose: Matrix form
        Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity(); // 初始化为单位矩阵

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

        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            // std::cout << serialized_data << std::endl << std::endl;
        };

        // Duration time of Trajectory
        time_duration = 10.0f;


        // Trajectory Planning
        // Polynomial interpolation of degree 5
        float t0 =0.0f; // use this value to define the time beginning of trajectory
        float T = time_duration;
        Eigen::MatrixXd para_matrix = traj.TrajectoryGeneration(T_Home_position, Final_pose, t0, T);
        Eigen::MatrixXd para_matrix_velocity = para_matrix.block<6,5>(0,1);


        int64_t initial_time = GetTickUs();
        now = initial_time;

        // Define running time of moving, t_running is 0 at beginning.
        double t_running = (now-initial_time)/MicrosecondToSeconds;

        // The independent variable in the trajectory: time
        double t_0;


        // Jacobian matrix
        Eigen::MatrixXd jacobian_matrix;
        Eigen::MatrixXd pseudo_inverse_jacobian_matrix;

        // Kinematic Calculation
        Eigen::Matrix4d current_FK;  // current homogeneous transformation matrix
        Eigen::VectorXd current_pose_interploration(6); // Expect position of Trajectory
        Eigen::Matrix3d current_Rotation;
        Eigen::Matrix3d Target_Rotation;

        // time Matrix
        Eigen::VectorXd time_matrix(6);


        // Error Calculation
        Eigen::VectorXd error(6);
        Eigen::Matrix3d rotation_error_matrix;
        Eigen::Vector3d rotation_error;

        // Actuator Incresement
        Eigen::VectorXd delta_q(7);

        // Frequency Calculation: Frequency = iteration number / time duration
        int iteration_number = 0;

        // Real-time loop
        while((t_running < T) && isConvergedAndWithLimit)
        {
            now = GetTickUs();
            if(last == 0) {
                now = abs(now);
            }else {
                t_running = (now-initial_time)/MicrosecondToSeconds;
            }

            if(now - last > 1000)  // 1ms  1毫秒
            {
                std::cout << "---------------" <<std::endl;
                std::cout << "Total Running time is: " << t_running <<std::endl;

                t_0 = t_running;

                // Calculate Jacobian and pseudo-inverse Jacobian
                jacobian_matrix = jacobian.computeJacobian(commands_rad);
                pseudo_inverse_jacobian_matrix = jacobian.computePseudoInverse(jacobian_matrix);

                // time Matrix
                time_matrix << 1, t_0, std::pow(t_0, 2), std::pow(t_0, 3), std::pow(t_0, 4), std::pow(t_0, 5);

                // Calculate Forward Kinematic, which is used to calculate error
                current_FK = fk.computeForwardKinematics(commands_rad);
                current_Rotation = current_FK.block<3, 3>(0, 0);

                // Expected position and orientation
                current_pose_interploration = para_matrix*time_matrix;
                Target_Rotation = fk.computeRotationMatrix(current_pose_interploration(3, 0),current_pose_interploration(4, 0),current_pose_interploration(5, 0));


                // Error Calculation
                // Rotation error - Axis Angle
                rotation_error_matrix = Target_Rotation * current_Rotation.transpose();
                Eigen::AngleAxisd rotation_error_angle_axis(rotation_error_matrix);
                rotation_error = rotation_error_angle_axis.angle() * rotation_error_angle_axis.axis();

                error(0) = current_pose_interploration(0, 0) - current_FK(0, 3);
                error(1) = current_pose_interploration(1, 0) - current_FK(1, 3);
                error(2) = current_pose_interploration(2, 0) - current_FK(2, 3);
                error(3) = rotation_error(0,0);
                error(4) = rotation_error(1,0);
                error(5) = rotation_error(2,0);


                // Actuator Incresement
                delta_q = pseudo_inverse_jacobian_matrix * (error);

                // Update Actuator position value
                for(int i = 0; i < actuator_count; i++)
                {
                    delta_q[i] = delta_q[i] * 180.0f/M_PI;

                    if (abs(delta_q[i])<1e-6) {
                        delta_q[i] = 0.0f;
                    }

                    commands[i] = delta_q[i] + commands[i];
                    base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));

                    commands_rad[i] = commands[i]*M_PI/180;
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
                iteration_number ++;
            }
        }

        std::cout << "interation number = " << iteration_number <<std::endl;

        // Computational forward kinematics
        std::cout << "actuator in radian: " << std::endl;
        for (size_t i = 0; i < commands_rad.size(); ++i) {
            std::cout << "actuator " << i+1 << " position in radian = " << commands_rad[i] << std::endl;
        }
        Eigen::Matrix4d T_final = fk.computeForwardKinematics(commands_rad);
        // Output the homogeneous transformation matrix of the forward kinematics
        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "Final Homogeneous Transformation Matrix:" << std::endl;
        std::cout << T_final << std::endl;

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
