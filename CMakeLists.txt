cmake_minimum_required(VERSION 2.8.3)
project(moveit_tests)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(
    catkin REQUIRED
        cmake_modules
        moveit_core
        moveit_msgs
        moveit_ros_move_group
        moveit_ros_planning
        moveit_ros_planning_interface
        roscpp)
find_package(Eigen REQUIRED)

catkin_package()

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(sync_box src/sync_box.cpp)
target_link_libraries(sync_box ${catkin_LIBRARIES})

add_executable(sync_mesh src/sync_mesh.cpp)
target_link_libraries(sync_mesh ${catkin_LIBRARIES})

add_executable(distance_field_test src/distance_field_test.cpp)
target_link_libraries(distance_field_test ${catkin_LIBRARIES})
