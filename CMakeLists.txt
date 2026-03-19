cmake_minimum_required(VERSION 3.8)
project(kinova_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(status_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

# pick and place
add_library(${PROJECT_NAME}_pick_place_task SHARED src/pick_place_task.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_task ${moveit_task_constructor_core_TARGETS})
install(TARGETS ${PROJECT_NAME}_pick_place_task
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME})
include_directories(include)

# move to config
add_library(${PROJECT_NAME}_move_to_config_task SHARED src/move_to_config_task.cpp)
target_link_libraries(${PROJECT_NAME}_move_to_config_task ${moveit_task_constructor_core_TARGETS})
install(TARGETS ${PROJECT_NAME}_move_to_config_task
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME})
include_directories(include)




# declare a demo consisting of a single cpp file
function(demo name)
	add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
	set(parameter_filename ${name}_parameters)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${parameter_filename}.yaml)
		generate_parameter_library(${parameter_filename} src/${parameter_filename}.yaml)
		target_link_libraries(${PROJECT_NAME}_${name} ${parameter_filename})
	endif()
	target_link_libraries(${PROJECT_NAME}_${name} ${moveit_task_constructor_core_TARGETS})
	set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
	install(TARGETS ${PROJECT_NAME}_${name}
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION lib/${PROJECT_NAME}
	)
endfunction()


# demo(cartesian)
# demo(modular)
# demo(alternative_path_costs)
# demo(ik_clearance_cost)
# demo(fallbacks_move_to)
# demo(move_to_config_task)
demo(pick_place_demo)
demo(manipulator_action_server) 

# target_link_libraries(${PROJECT_NAME}_pick_place_task pick_place_demo_parameters)
target_link_libraries(${PROJECT_NAME}_pick_place_task manipulator_action_server_parameters)
target_link_libraries(${PROJECT_NAME}_move_to_config_task manipulator_action_server_parameters)
target_link_libraries(${PROJECT_NAME}_pick_place_demo ${PROJECT_NAME}_pick_place_task)
# target_link_libraries(${PROJECT_NAME}_manipulator_action_server ${PROJECT_NAME}_pick_place_task )

target_link_libraries(${PROJECT_NAME}_manipulator_action_server
  ${PROJECT_NAME}_pick_place_task
  ${PROJECT_NAME}_move_to_config_task
  manipulator_action_server_parameters
)

ament_target_dependencies(${PROJECT_NAME}_manipulator_action_server 
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_task_constructor_core
  status_interfaces
)

# arm server
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

install(TARGETS
  arm_server  
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

# manipualtor action server
# Define the Action Server executable
# add_executable(manipulator_action_server src/manipulator_action_server.cpp)
# # Target dependencies (linking libraries)
# target_link_libraries(manipulator_action_server
#   PUBLIC
#   # This links to the logic in pick_place_task.h/cpp if it's in this same package
#   # If pick_place_task is a separate library, add it here.
# )
# ament_target_dependencies(manipulator_action_server 
#   rclcpp 
#   rclcpp_action 
#   moveit_ros_planning_interface 
#   moveit_task_constructor_core
#   status_interfaces
#   tf2_ros
#   tf2_geometry_msgs  # Added
#   tf2_eigen          # Added
# )

# # Install
# install(TARGETS
#   manipulator_action_server
#   DESTINATION lib/${PROJECT_NAME}
# )

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
