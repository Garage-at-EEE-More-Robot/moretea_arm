cmake_minimum_required(VERSION 3.8)
project(moretea_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlayTrajectory.action"
)

# Include directories for SCServo_Linux
include_directories(
  include
)

add_executable(play_trajectory_server
  src/play_trajectory_action_node.cpp
  src/servo_backend.cpp
)

ament_target_dependencies(play_trajectory_server
  rclcpp
  rclcpp_action
  ament_index_cpp
)

# Link against SCServo library and yaml-cpp
target_link_libraries(play_trajectory_server 
  yaml-cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/SCServo_Linux/build/libSCServo.a
)

# Link against the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(play_trajectory_server "${cpp_typesupport_target}")

# IMPORTANT: Add dependency to ensure action is generated before executable
rosidl_target_interfaces(play_trajectory_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS play_trajectory_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(rosidl_default_runtime)

ament_package()
