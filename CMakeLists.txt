cmake_minimum_required(VERSION 2.8.3)
project(moveit_scenarios)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED)

find_package(
    catkin REQUIRED
        actionlib
        cmake_modules
        leatherman
        moveit_core
        moveit_msgs
        moveit_ros_move_group
        moveit_ros_planning
        moveit_ros_planning_interface
        roscpp
        sbpl_geometry_utils
        smpl_ros
        visualization_msgs)
find_package(Eigen3 REQUIRED)
find_package(smpl REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(yaml REQUIRED yaml-cpp)

catkin_package()

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${yaml_INCLUDE_DIRS})

add_executable(sync_box src/sync_box.cpp)
target_link_libraries(sync_box ${catkin_LIBRARIES})

add_executable(sync_mesh src/sync_mesh.cpp)
target_link_libraries(sync_mesh ${catkin_LIBRARIES})

add_executable(distance_field_test src/distance_field_test.cpp)
target_link_libraries(distance_field_test ${catkin_LIBRARIES})

add_executable(
    sample_birdhouse_goals
    src/sample_birdhouse_goals.cpp
    src/birdhouse_test_suite.cpp)
target_link_libraries(sample_birdhouse_goals ${catkin_LIBRARIES} smpl::smpl)

add_executable(compute_torque_manifold src/compute_torque_manifold.cpp)
target_link_libraries(compute_torque_manifold ${catkin_LIBRARIES} smpl::smpl)

add_executable(torque_manifold_bb src/torque_manifold_bb.cpp)

add_executable(apc_test_suite src/apc_test_suite.cpp)
target_link_libraries(apc_test_suite ${catkin_LIBRARIES} smpl::smpl)

add_executable(run_moveit_tests src/run_moveit_tests.cpp)
target_link_libraries(
    run_moveit_tests
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${yaml_LIBRARIES})
