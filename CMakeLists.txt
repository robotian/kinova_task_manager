cmake_minimum_required(VERSION 3.8)
project(kinova_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(status_interfaces REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(arm_server src/arm_server.cpp)

# Tell CMake where to find the headers for this executable
target_include_directories(arm_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_server 
  rclcpp 
  rclcpp_action 
  moveit_ros_planning_interface 
  moveit_task_constructor_core
  status_interfaces
  tf2_ros
  tf2_geometry_msgs  # Added
  tf2_eigen          # Added
)

# 3. Define the Client
# add_executable(arm_client src/arm_client.cpp)
# ament_target_dependencies(arm_client 
#   rclcpp 
#   rclcpp_action 
#   status_interfaces
# )

install(TARGETS
  arm_server  
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

# Export the include directory for other packages
ament_export_include_directories(include)

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


ament_package()
