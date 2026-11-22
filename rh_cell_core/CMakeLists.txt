cmake_minimum_required(VERSION 3.16)
project(rh_cell_core)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Plain C++17 library. Deliberately NO ROS includes: this code must run anywhere.
add_library(rh_cell_core src/admittance_1d.cpp src/force_guard.cpp src/insertion_monitor.cpp src/virtual_fixture.cpp
            src/cartesian_admittance.cpp src/peg_contact_2d.cpp)
target_compile_features(rh_cell_core PUBLIC cxx_std_17)
target_compile_options(rh_cell_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rh_cell_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rh_cell_core PUBLIC Eigen3::Eigen)

add_executable(contact_demo src/contact_demo.cpp)
target_link_libraries(contact_demo rh_cell_core)
# RH Cell 2.0 step 2: planar chamfer model, stiff vs compliant, friction sweep (no Gazebo)
add_executable(chamfer_demo src/chamfer_demo.cpp)
target_link_libraries(chamfer_demo rh_cell_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rh_cell_core EXPORT export_rh_cell_core
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS contact_demo chamfer_demo DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_rh_cell_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core rh_cell_core)
endif()

ament_package()
