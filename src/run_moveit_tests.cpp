////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

// standard includes
#include <cstdlib>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <boost/filesystem.hpp>
#include <leatherman/print.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <ros/ros.h>

bool OverwriteGoal(
    const boost::filesystem::path& scenario_filepath,
    moveit_msgs::MoveGroupGoal& goal)
{
    ROS_INFO_STREAM("Overwriting config from " << scenario_filepath);
    return true;
}

bool ParseGoal(
    const boost::filesystem::path& base_scenario_path,
    const boost::filesystem::path& scenario_path,
    moveit_msgs::MoveGroupGoal& goal)
{
    boost::filesystem::path abs_base_path =
            boost::filesystem::absolute(base_scenario_path);

    boost::filesystem::path abs_path =
            boost::filesystem::absolute(scenario_path);

    ROS_DEBUG_STREAM("Absolute Base Path: " << abs_base_path);
    ROS_DEBUG_STREAM("Absolute Path: " << abs_path);

    // check that scenario_path is an ancestor of base_scenario_path
    const std::string base_path_str = abs_base_path.generic_string();
    const std::string path_str = abs_path.generic_string();
    if (path_str.find(base_path_str) != 0) {
        ROS_ERROR_STREAM("base scenario path must be an ancestor of scenario path");
        return false;
    }

    std::vector<boost::filesystem::path> base_path_entries;
    std::vector<boost::filesystem::path> path_entries;

    for (auto it = abs_base_path.begin();
            it != abs_base_path.end(); ++it)
    {
        base_path_entries.push_back(*it);
    }

    for (auto it = abs_path.begin(); it != abs_path.end(); ++it) {
        path_entries.push_back(*it);
    }

    path_entries.erase(
            path_entries.begin(),
            path_entries.begin() + base_path_entries.size());
    base_path_entries.clear();

    ROS_INFO_STREAM("Path entries: " << path_entries);

    boost::filesystem::path p = base_scenario_path;
    bool done = false;
    while (!done) {
        boost::filesystem::path scenario_filepath = p / "scenario.yaml";
        if (boost::filesystem::exists(scenario_filepath) &&
            boost::filesystem::is_regular_file(scenario_filepath))
        {
            OverwriteGoal(scenario_filepath, goal);
        }

        if (!path_entries.empty()) {
            p /= path_entries.front();
            path_entries.erase(path_entries.begin());
        }
        else {
            done = true;
        }
    }

    return true;
}

void PrintPathInfo(const boost::filesystem::path& p)
{
  ROS_INFO_STREAM("root_name(): "           << p.root_name());
  ROS_INFO_STREAM("root_directory(): "      << p.root_directory());
  ROS_INFO_STREAM("root_path(): "           << p.root_path());
  ROS_INFO_STREAM("relative_path(): "       << p.relative_path());
  ROS_INFO_STREAM("parent_path(): "         << p.parent_path());
  ROS_INFO_STREAM("filename(): "            << p.filename());
  ROS_INFO_STREAM("stem(): "                << p.stem());
  ROS_INFO_STREAM("extension(): "           << p.extension());
  ROS_INFO_STREAM("empty(): "               << p.empty());
  ROS_INFO_STREAM("is_absolute(): "         << p.is_absolute());
  ROS_INFO_STREAM("has_root_name(): "       << p.has_root_name());
  ROS_INFO_STREAM("has_root_directory(): "  << p.has_root_directory());
  ROS_INFO_STREAM("has_root_path(): "       << p.has_root_path());
  ROS_INFO_STREAM("has_relative_path(): "   << p.has_relative_path());
  ROS_INFO_STREAM("has_parent_path(): "     << p.has_parent_path());
  ROS_INFO_STREAM("has_filename(): "        << p.has_filename());
  ROS_INFO_STREAM("has_stem(): "            << p.has_stem());
  ROS_INFO_STREAM("has_extension(): "       << p.has_extension());
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

    // clean up base and derived paths, if files are given for either, the
    // parent directory is used

    boost::filesystem::path base_scenario_path(base_scenario_dir);
    if (!boost::filesystem::exists(base_scenario_path)) {
        std::cerr << base_scenario_path << " does not exist" << std::endl;
        return 1;
    }
    if (!boost::filesystem::is_directory(base_scenario_path)) {
        base_scenario_path = base_scenario_path.parent_path();
    }

    const char* scenario_dir = argv[1];
    boost::filesystem::path scenario_path(scenario_dir);
    if (!boost::filesystem::exists(scenario_path)) {
        std::cerr << scenario_path << " does not exist" << std::endl;
        return 1;
    }
    if (!boost::filesystem::is_directory(scenario_path)) {
        scenario_path = scenario_path.parent_path();
    }

    // parse the move group goal from hierarchical config

    moveit_msgs::MoveGroupGoal goal;
    if (!ParseGoal(base_scenario_path, scenario_path, goal)) {
        ROS_ERROR("Failed to parse goal");
        return 1;
    }

    typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>
    MoveGroupActionClient;

    MoveGroupActionClient move_group_client("move_group", true);

    ROS_DEBUG("Waiting for Move Group Action Server");
    move_group_client.waitForServer();

    ROS_DEBUG("Sending Move Group Goal");
    move_group_client.sendGoal(goal);

    ROS_DEBUG("Waiting for result");
    move_group_client.waitForResult();

    if (move_group_client.getState() ==
            actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_DEBUG("Result received");
    }

    return 0;
}
