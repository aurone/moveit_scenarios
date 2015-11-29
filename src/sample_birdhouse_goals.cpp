#include <ros/ros.h>

#include "birdhouse_test_suite.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sample_birdhouse_goals");

    BirdhouseTestSuite tests;
    if (!tests.init()) {
        ROS_ERROR("Failed to initialize Birdhouse Test Suite");
        return 1;
    }

    return tests.run();
}
