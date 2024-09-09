cmake_minimum_required(VERSION 3.8)
project(kinematic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(std_msgs REQUIRED)  # Ensure std_msgs is found
find_package(sensor_msgs REQUIRED)




# Define executables
add_executable(ForwardKinematic src/ForwardKinematic.cpp)
add_executable(Control src/Control.cpp)

# Set include directories and compile features
target_include_directories(ForwardKinematic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})  # Ensure Eigen include directory is added

target_compile_features(ForwardKinematic PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link dependencies to the target
ament_target_dependencies(Control
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp)

ament_target_dependencies(ForwardKinematic
  rclcpp
  Eigen3
  sensor_msgs)  # Link std_msgs to the ForwardKinematic target

  

  
# Install targets
install(TARGETS ForwardKinematic Control
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
