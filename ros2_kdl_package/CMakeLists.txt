cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ros2_kdl_vision_control src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp src/ros2_kdl_vision_control.cpp )

# Include directories
target_include_directories(ros2_kdl_vision_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_vision_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# Add dependencies
ament_target_dependencies( 
  ros2_kdl_vision_control
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS ros2_kdl_vision_control
  DESTINATION lib/${PROJECT_NAME})

# Run tests if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
