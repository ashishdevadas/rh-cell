cmake_minimum_required(VERSION 3.16)
project(rh_cell_tasks)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rh_cell_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(insertion_sequence src/insertion_sequence.cpp)
target_compile_features(insertion_sequence PUBLIC cxx_std_17)
ament_target_dependencies(insertion_sequence
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
)

# F/T front-end check (Step 3c-2a)
add_executable(ft_monitor src/ft_monitor.cpp)
target_compile_features(ft_monitor PUBLIC cxx_std_17)
target_include_directories(ft_monitor PRIVATE include)
target_link_libraries(ft_monitor rh_cell_core::rh_cell_core)
ament_target_dependencies(ft_monitor rclcpp geometry_msgs)

# Force-controlled insertion (Step 3c-2b)
add_executable(force_insertion src/force_insertion.cpp src/cell.cpp src/cartesian_streamer.cpp)
target_compile_features(force_insertion PUBLIC cxx_std_17)
target_include_directories(force_insertion PRIVATE include)
target_link_libraries(force_insertion rh_cell_core::rh_cell_core)
ament_target_dependencies(force_insertion
  rclcpp moveit_core moveit_ros_planning_interface geometry_msgs moveit_msgs shape_msgs std_msgs
  sensor_msgs controller_manager_msgs)

# Teleoperation with a virtual fixture (Phase 1.3)
add_executable(teleop_fixture src/teleop_fixture.cpp src/cell.cpp src/cartesian_streamer.cpp)
target_compile_features(teleop_fixture PUBLIC cxx_std_17)
target_include_directories(teleop_fixture PRIVATE include)
target_link_libraries(teleop_fixture rh_cell_core::rh_cell_core)
ament_target_dependencies(teleop_fixture
  rclcpp moveit_core moveit_ros_planning_interface geometry_msgs moveit_msgs shape_msgs std_msgs
  sensor_msgs controller_manager_msgs)

install(TARGETS insertion_sequence ft_monitor force_insertion teleop_fixture DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/plot_run.py scripts/plot_faults.py scripts/plot_teleop.py scripts/teleop_demo.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
