// standard includes
#include <cstdlib>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <boost/filesystem.hpp>
#include <moveit_msgs/MoveGroupAction.h>
#include <ros/ros.h>

bool ParseGoal(
    const boost::filesystem::path& base_scenario_path,
    const boost::filesystem::path& scenario_path,
    moveit_msgs::MoveGroupGoal& goal)
{
    // TODO: implement
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "run_moveit_tests");
    ros::NodeHandle nh;

    char* base_scenario_dir = std::getenv("SCENARIO_DIR");
    if (!base_scenario_dir) {
        ROS_ERROR("Requires SCENARIO_DIR environment variable to be set");
        return 1;
    }

    if (argc < 2) {
        ROS_ERROR("Usage: run_moveit_tests <scenario path>");
        return 1;
    }

    boost::filesystem::path base_scenario_path(base_scenario_dir);

    const char* scenario_dir = argv[1];
    boost::filesystem::path scenario_path(scenario_dir);

    moveit_msgs::MoveGroupGoal goal;
    if (!ParseGoal(base_scenario_path, scenario_path, goal)) {
        ROS_ERROR("Failed to parse goal");
        return 1;
    }

    typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>
    MoveGroupActionClient;

    MoveGroupActionClient move_group_client("move_group", true);
    move_group_client.waitForServer();

    move_group_client.sendGoal(goal);
    move_group_client.waitForResult();

    if (move_group_client.getState() ==
            actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Result received");
    }

    return 0;
}
