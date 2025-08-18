cmake_minimum_required(VERSION 3.8)
project(ffw_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# C++ executables
add_executable(kdl_ik_solver_cpp src/kdl_ik_solver.cpp)

target_include_directories(kdl_ik_solver_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(kdl_ik_solver_cpp PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(kdl_ik_solver_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  urdf
  orocos_kdl
)

# Realtime IK Solver executable
add_executable(realtime_ik_solver src/realtime_ik_solver.cpp)

target_include_directories(realtime_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(realtime_ik_solver PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(realtime_ik_solver
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  urdf
  orocos_kdl
)

# Target Pose Publisher executable
add_executable(target_pose_publisher src/target_pose_publisher.cpp)

target_include_directories(target_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(target_pose_publisher PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(target_pose_publisher
  rclcpp
  geometry_msgs
)

# IK Solver Comparison executable
add_executable(ik_solver_comparison src/ik_solver_comparison.cpp)

target_include_directories(ik_solver_comparison PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(ik_solver_comparison PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ik_solver_comparison
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  kdl_parser
  urdf
  orocos_kdl
)

# Realtime IK Solver with Joint Limits executable
add_executable(realtime_ik_solver_jl src/realtime_ik_solver_jl.cpp)

target_include_directories(realtime_ik_solver_jl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(realtime_ik_solver_jl PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(realtime_ik_solver_jl
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  urdf
  orocos_kdl
)

# Realtime IK Solver Arm Only executable
add_executable(realtime_ik_solver_arm_only src/realtime_ik_solver_arm_only.cpp)

target_include_directories(realtime_ik_solver_arm_only PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(realtime_ik_solver_arm_only PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(realtime_ik_solver_arm_only
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  urdf
  orocos_kdl
)

# Realtime IK Solver Interpolation executable
add_executable(realtime_ik_solver_interpolation src/realtime_ik_solver_interpolation.cpp)

target_include_directories(realtime_ik_solver_interpolation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(realtime_ik_solver_interpolation PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(realtime_ik_solver_interpolation
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  urdf
  orocos_kdl
)

# VR Safe IK Solver executable
add_executable(vr_safe_ik_solver src/vr_safe_ik_solver.cpp)

target_include_directories(vr_safe_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(vr_safe_ik_solver PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(vr_safe_ik_solver
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  urdf
  orocos_kdl
)

# Pinocchio IK Solver executable
add_executable(pinocchio_ik_solver src/pinocchio_ik_solver.cpp)

target_include_directories(pinocchio_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pinocchio_ik_solver PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(pinocchio_ik_solver
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  urdf
)


# VR Hand Pose Transformer executable
add_executable(vr_hand_pose_transformer src/vr_hand_pose_transformer.cpp)

target_include_directories(vr_hand_pose_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(vr_hand_pose_transformer PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(vr_hand_pose_transformer
  rclcpp
  geometry_msgs
)

# Link Pinocchio manually (since it might not have proper CMake integration in ROS2)
target_link_libraries(pinocchio_ik_solver pinocchio::pinocchio)

# Link Eigen3 to VR hand pose transformer
target_link_libraries(vr_hand_pose_transformer Eigen3::Eigen)

# Install C++ executables
install(TARGETS kdl_ik_solver_cpp realtime_ik_solver target_pose_publisher ik_solver_comparison realtime_ik_solver_jl realtime_ik_solver_arm_only realtime_ik_solver_interpolation vr_safe_ik_solver pinocchio_ik_solver vr_hand_pose_transformer
  DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
