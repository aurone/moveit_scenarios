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
#include <fstream>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <boost/filesystem.hpp>
#include <leatherman/print.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

bool Overwrite(std::uint8_t& b, YAML::Node n);
//bool Overwrite(std::int8_t& b, YAML::Node n);
bool Overwrite(std::int32_t& i, YAML::Node n);
bool Overwrite(double& d, YAML::Node n);
bool Overwrite(std::string& o, YAML::Node n);
bool Overwrite(std::uint32_t& o, YAML::Node n);
bool Overwrite(ros::Time& o, YAML::Node n);
bool Overwrite(geometry_msgs::Vector3& o, YAML::Node n);
bool Overwrite(geometry_msgs::Quaternion& o, YAML::Node n);
bool Overwrite(geometry_msgs::Transform& o, YAML::Node n);
bool Overwrite(geometry_msgs::Twist& o, YAML::Node n);
bool Overwrite(geometry_msgs::Wrench& o, YAML::Node n);
bool Overwrite(geometry_msgs::Point& o, YAML::Node n);
bool Overwrite(geometry_msgs::Pose& o, YAML::Node n);
bool Overwrite(std_msgs::Header& o, YAML::Node n);
bool Overwrite(sensor_msgs::JointState& o, YAML::Node n);
bool Overwrite(sensor_msgs::MultiDOFJointState& o, YAML::Node n);
bool Overwrite(trajectory_msgs::JointTrajectory& o, YAML::Node n);
bool Overwrite(moveit_msgs::CollisionObject& o, YAML::Node n);
bool Overwrite(moveit_msgs::AttachedCollisionObject& o, YAML::Node n);
bool Overwrite(moveit_msgs::WorkspaceParameters& o, YAML::Node n);
bool Overwrite(moveit_msgs::PlanningScene& o, YAML::Node n);
bool Overwrite(moveit_msgs::RobotState& o, YAML::Node n);
bool Overwrite(moveit_msgs::BoundingVolume& o, YAML::Node n);
bool Overwrite(moveit_msgs::JointConstraint& o, YAML::Node n);
bool Overwrite(moveit_msgs::PositionConstraint& o, YAML::Node n);
bool Overwrite(moveit_msgs::OrientationConstraint& o, YAML::Node n);
bool Overwrite(moveit_msgs::VisibilityConstraint& o, YAML::Node n);
bool Overwrite(moveit_msgs::Constraints& o, YAML::Node n);
bool Overwrite(moveit_msgs::TrajectoryConstraints& o, YAML::Node n);
bool Overwrite(moveit_msgs::MotionPlanRequest& o, YAML::Node n);
bool Overwrite(moveit_msgs::PlanningOptions& o, YAML::Node n);

template <typename T>
bool Overwrite(std::vector<T>& v, YAML::Node n)
{
    for (auto it = n.begin(); it != n.end(); ++it) {
        v.emplace_back();
        YAML::Node nn = *it;
        Overwrite(v.back(), nn);
    }
    return true;
}

//////////////////

bool Overwrite(std::uint8_t& b, YAML::Node n)
{
    b = n.as<bool>();
    return true;
}

//bool Overwrite(std::int8_t& b, YAML::Node n)
//{
//    b = n.as<std::int8_t>();
//    return true;
//}

bool Overwrite(std::int32_t& i, YAML::Node n)
{
    i = n.as<std::int32_t>();
    return true;
}

bool Overwrite(double& d, YAML::Node n)
{
    d = n.as<double>();
    return true;
}

bool Overwrite(std::string& o, YAML::Node n)
{
    o = n.as<std::string>();
    return true;
}

bool Overwrite(std::uint32_t& o, YAML::Node n)
{
    o = n.as<std::uint32_t>();
    return true;
}

bool Overwrite(ros::Time& o, YAML::Node n)
{
    o = ros::Time(n.as<std::int32_t>() / 1000000000, n.as<std::int32_t>() % 1000000000);
}

bool Overwrite(geometry_msgs::Vector3& o, YAML::Node n)
{
    if (n["x"]) {
        Overwrite(o.x, n["x"]);
    }
    if (n["y"]) {
        Overwrite(o.y, n["y"]);
    }
    if (n["z"]) {
        Overwrite(o.z, n["z"]);
    }
    return true;
}

bool Overwrite(geometry_msgs::Quaternion& o, YAML::Node n)
{
    if (n["w"]) {
        Overwrite(o.w, n["w"]);
    }
    if (n["x"]) {
        Overwrite(o.x, n["x"]);
    }
    if (n["y"]) {
        Overwrite(o.y, n["y"]);
    }
    if (n["z"]) {
        Overwrite(o.z, n["z"]);
    }
    return true;
}

bool Overwrite(geometry_msgs::Transform& o, YAML::Node n)
{
    if (n["translation"]) {
        Overwrite(o.translation, n["translation"]);
    }
    if (n["rotation"]) {
        Overwrite(o.rotation, n["rotation"]);
    }
    return true;
}

bool Overwrite(geometry_msgs::Twist& o, YAML::Node n)
{
    if (n["linear"]) {
        Overwrite(o.linear, n["linear"]);
    }
    if (n["angular"]) {
        Overwrite(o.angular, n["angular"]);
    }
    return true;
}

bool Overwrite(geometry_msgs::Wrench& o, YAML::Node n)
{
    if (n["force"]) {
        Overwrite(o.force, n["force"]);
    }
    if (n["torque"]) {
        Overwrite(o.torque, n["torque"]);
    }
    return true;
}

bool Overwrite(geometry_msgs::Point& o, YAML::Node n)
{
    if (n["x"]) {
        Overwrite(o.x, n["x"]);
    }
    if (n["y"]) {
        Overwrite(o.y, n["y"]);
    }
    if (n["z"]) {
        Overwrite(o.z, n["z"]);
    }
    return true;
}

bool Overwrite(geometry_msgs::Pose& o, YAML::Node n)
{
    if (n["position"]) {
        Overwrite(o.position, n["position"]);
    }
    if (n["orientation"]) {
        Overwrite(o.orientation, n["orientation"]);
    }
    return true;
}

bool Overwrite(std_msgs::Header& o, YAML::Node n)
{
    if (n["seq"]) {
        Overwrite(o.seq, n["seq"]);
    }
    if (n["stamp"]) {
        Overwrite(o.stamp, n["stamp"]);
    }
    if (n["frame_id"]) {
        Overwrite(o.frame_id, n["frame_id"]);
    }
    return true;
}

bool Overwrite(sensor_msgs::JointState& o, YAML::Node n)
{
    if (n["header"]) {
        Overwrite(o.header, n["header"]);
    }
    if (n["name"]) {
        Overwrite(o.name, n["name"]);
    }
    if (n["position"]) {
        Overwrite(o.position, n["position"]);
    }
    if (n["velocity"]) {
        Overwrite(o.velocity, n["velocity"]);
    }
    if (n["effort"]) {
        Overwrite(o.effort, n["effort"]);
    }
    return true;
}

bool Overwrite(sensor_msgs::MultiDOFJointState& o, YAML::Node n)
{
    if (n["header"]) {
        Overwrite(o.header, n["header"]);
    }
    if (n["joint_names"]) {
        Overwrite(o.joint_names, n["joint_names"]);
    }
    if (n["transforms"]) {
        Overwrite(o.transforms, n["transforms"]);
    }
    if (n["twist"]) {
        Overwrite(o.twist, n["twist"]);
    }
    if (n["wrench"]) {
        Overwrite(o.wrench, n["wrench"]);
    }
    return true;
}

bool Overwrite(trajectory_msgs::JointTrajectory& o, YAML::Node n)
{
    ROS_ERROR_ONCE("%s unimplemented", __PRETTY_FUNCTION__);
    return true;
}

bool Overwrite(moveit_msgs::CollisionObject& o, YAML::Node n)
{
    ROS_ERROR_ONCE("%s unimplemented", __PRETTY_FUNCTION__);
    return true;
}

bool Overwrite(moveit_msgs::AttachedCollisionObject& o, YAML::Node n)
{
    if (n["link_name"]) {
        Overwrite(o.link_name, n["link_name"]);
    }
    if (n["object"]) {
        Overwrite(o.object, n["object"]);
    }
    if (n["touch_links"]) {
        Overwrite(o.touch_links, n["touch_links"]);
    }
    if (n["detach_posture"]) {
        Overwrite(o.detach_posture, n["detach_posture"]);
    }
    if (n["weight"]) {
        Overwrite(o.weight, n["weight"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::WorkspaceParameters& o, YAML::Node n)
{
    if (n["header"]) {
        Overwrite(o.header, n["header"]);
    }
    if (n["min_corner"]) {
        Overwrite(o.min_corner, n["min_corner"]);
    }
    if (n["max_corner"]) {
        Overwrite(o.max_corner, n["max_corner"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::PlanningScene& o, YAML::Node n)
{
    return true;
}

bool Overwrite(moveit_msgs::RobotState& o, YAML::Node n)
{
    if (n["joint_state"]) {
        Overwrite(o.joint_state, n["joint_state"]);
    }
    if (n["multi_dof_joint_state"]) {
        Overwrite(o.multi_dof_joint_state, n["multi_dof_joint_state"]);
    }
    if (n["attached_collision_objects"]) {
        Overwrite(o.attached_collision_objects, n["attached_collision_objects"]);
    }
    if (n["is_diff"]) {
        Overwrite(o.is_diff, n["is_diff"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::BoundingVolume& o, YAML::Node n)
{
    ROS_ERROR_ONCE("%s unimplemented", __PRETTY_FUNCTION__);
    return true;
}

bool Overwrite(moveit_msgs::JointConstraint& o, YAML::Node n)
{
    if (n["joint_name"]) {
        Overwrite(o.joint_name, n["joint_name"]);
    }
    if (n["position"]) {
        Overwrite(o.position, n["position"]);
    }
    if (n["tolerance_above"]) {
        Overwrite(o.tolerance_above, n["tolerance_above"]);
    }
    if (n["tolerance_below"]) {
        Overwrite(o.tolerance_below, n["tolerance_below"]);
    }
    if (n["weight"]) {
        Overwrite(o.weight, n["weight"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::PositionConstraint& o, YAML::Node n)
{
    if (n["header"]) {
        Overwrite(o.header, n["header"]);
    }
    if (n["link_name"]) {
        Overwrite(o.link_name, n["link_name"]);
    }
    if (n["target_point_offset"]) {
        Overwrite(o.target_point_offset, n["target_point_offset"]);
    }
    if (n["constraint_region"]) {
        Overwrite(o.constraint_region, n["constraint_region"]);
    }
    if (n["weight"]) {
        Overwrite(o.weight, n["weight"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::OrientationConstraint& o, YAML::Node n)
{
    if (n["header"]) {
        Overwrite(o.header, n["header"]);
    }
    if (n["orientation"]) {
        Overwrite(o.orientation, n["orientation"]);
    }
    if (n["link_name"]) {
        Overwrite(o.link_name, n["link_name"]);
    }
    if (n["absolute_x_axis_tolerance"]) {
        Overwrite(o.absolute_x_axis_tolerance, n["absolute_x_axis_tolerance"]);
    }
    if (n["absolute_y_axis_tolerance"]) {
        Overwrite(o.absolute_y_axis_tolerance, n["absolute_y_axis_tolerance"]);
    }
    if (n["absolute_z_axis_tolerance"]) {
        Overwrite(o.absolute_z_axis_tolerance, n["absolute_z_axis_tolerance"]);
    }
    if (n["weight"]) {
        Overwrite(o.weight, n["weight"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::VisibilityConstraint& o, YAML::Node n)
{
    ROS_ERROR_ONCE("%s unimplemented", __PRETTY_FUNCTION__);
    return true;
}

bool Overwrite(moveit_msgs::Constraints& o, YAML::Node n)
{
    if (n["name"]) {
        Overwrite(o.name, n["name"]);
    }
    if (n["joint_constraints"]) {
        Overwrite(o.joint_constraints, n["joint_constraints"]);
    }
    if (n["position_constraints"]) {
        Overwrite(o.position_constraints, n["position_constraints"]);
    }
    if (n["orientation_constraints"]) {
        Overwrite(o.orientation_constraints, n["orientation_constraints"]);
    }
    if (n["visibility_constraints"]) {
        Overwrite(o.visibility_constraints, n["visibility_constraints"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::TrajectoryConstraints& o, YAML::Node n)
{
    if (n["constraints"]) {
        Overwrite(o.constraints, n["constraints"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::MotionPlanRequest& o, YAML::Node n)
{
    if (n["workspace_parameters"]) {
        Overwrite(o.workspace_parameters, n["workspace_parameters"]);
    }
    if (n["start_state"]) {
        Overwrite(o.start_state, n["start_state"]);
    }
    if (n["goal_constraints"]) {
        Overwrite(o.goal_constraints, n["goal_constraints"]);
    }
    if (n["path_constraints"]) {
        Overwrite(o.path_constraints, n["path_constraints"]);
    }
    if (n["trajectory_constraints"]) {
        Overwrite(o.trajectory_constraints, n["trajectory_constraints"]);
    }
    if (n["planner_id"]) {
        Overwrite(o.planner_id, n["planner_id"]);
    }
    if (n["group_name"]) {
        Overwrite(o.group_name, n["group_name"]);
    }
    if (n["num_planning_attempts"]) {
        Overwrite(o.num_planning_attempts, n["num_planning_attempts"]);
    }
    if (n["allowed_planning_time"]) {
        Overwrite(o.allowed_planning_time, n["allowed_planning_time"]);
    }
    if (n["max_velocity_scaling_factor"]) {
        Overwrite(o.max_velocity_scaling_factor, n["max_velocity_scaling_factor"]);
    }
    if (n["max_acceleration_scaling_factor"]) {
        Overwrite(o.max_acceleration_scaling_factor, n["max_acceleration_scaling_factor"]);
    }
    return true;
}

bool Overwrite(moveit_msgs::PlanningOptions& o, YAML::Node n)
{
    if (n["planning_scene_diff"]) {
        Overwrite(o.planning_scene_diff, n["planning_scene_diff"]);
    }
    if (n["plan_only"]) {
        Overwrite(o.plan_only, n["plan_only"]);
    }
    if (n["look_around"]) {
        Overwrite(o.look_around, n["look_around"]);
    }
    if (n["look_around_attempts"]) {
        Overwrite(o.look_around_attempts, n["look_around_attempts"]);
    }
    if (n["max_safe_execution_cost"]) {
        Overwrite(o.max_safe_execution_cost, n["max_safe_execution_cost"]);
    }
    if (n["replan"]) {
        Overwrite(o.replan, n["replan"]);
    }
    if (n["replan_attempts"]) {
        Overwrite(o.replan_attempts, n["replan_attempts"]);
    }
    if (n["replan_delay"]) {
        Overwrite(o.replan_delay, n["replan_delay"]);
    }

    return true;
}

bool OverwriteGoal(
    const boost::filesystem::path& scenario_filepath,
    moveit_msgs::MoveGroupGoal& goal)
{
    ROS_INFO_STREAM("merging config from " << scenario_filepath);
    std::ifstream ifs(scenario_filepath.c_str());
    YAML::Node config = YAML::LoadFile(scenario_filepath.c_str());

    if (config["request"]) {
        Overwrite(goal.request, config["request"]);
    }
    if (config["planning_options"]) {
        Overwrite(goal.planning_options, config["planning_options"]);
    }

    return true;
}

bool ParseGoal(
    const boost::filesystem::path& root_scenario_path,
    const boost::filesystem::path& scenario_path,
    moveit_msgs::MoveGroupGoal& goal)
{
    ROS_INFO_STREAM("Absolute Base Path: " << root_scenario_path);
    ROS_INFO_STREAM("Absolute Path: " << scenario_path);

    // check that scenario_path is an ancestor of root_scenario_path
    const std::string base_path_str = root_scenario_path.generic_string();
    const std::string path_str = scenario_path.generic_string();
    if (path_str.find(base_path_str) != 0) {
        ROS_ERROR_STREAM("base scenario path must be an ancestor of scenario path");
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // get the sequence path suffixes from the root directory to the scenario //
    // directory                                                              //
    ////////////////////////////////////////////////////////////////////////////

    std::vector<boost::filesystem::path> path_entries;
    {
        std::vector<boost::filesystem::path> base_path_entries;

        std::cout << "base path entries:" << std::endl;
        for (auto it = root_scenario_path.begin();
                it != root_scenario_path.end(); ++it)
        {
            std::cout << *it << std::endl;
            base_path_entries.push_back(*it);
        }

        std::cout << "path entries:" << std::endl;
        for (auto it = scenario_path.begin(); it != scenario_path.end(); ++it) {
            std::cout << *it << std::endl;
            path_entries.push_back(*it);
        }

        path_entries.erase(
                path_entries.begin(),
                path_entries.begin() + base_path_entries.size());
        base_path_entries.clear();
    }

    ROS_INFO_STREAM("trailing path entries: " << path_entries);

    boost::filesystem::path p = root_scenario_path;
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

    if (argc < 2) {
        ROS_ERROR("Usage: run_moveit_tests <scenario path>");
        return 1;
    }

    const char* scenario_root_var = "SCENARIO_ROOT";
    char* base_scenario_dir = std::getenv(scenario_root_var);
    if (!base_scenario_dir) {
        ROS_ERROR("Requires '%s' environment variable to be set", scenario_root_var);
        return 1;
    }

    // prepare root and scenario paths
    // -> check for existence
    // -> file -> parent directory
    // -> directory path -> absolute directory path

    boost::filesystem::path root_scenario_path(base_scenario_dir);
    if (!boost::filesystem::exists(root_scenario_path)) {
        std::cerr << root_scenario_path << " does not exist" << std::endl;
        return 1;
    }
    if (!boost::filesystem::is_directory(root_scenario_path)) {
        root_scenario_path = root_scenario_path.parent_path();
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
    root_scenario_path = boost::filesystem::absolute(root_scenario_path);
    scenario_path = boost::filesystem::absolute(scenario_path);

    moveit_msgs::MoveGroupGoal goal;
    if (!ParseGoal(root_scenario_path, scenario_path, goal)) {
        ROS_ERROR("Failed to parse goal");
        return 1;
    }

    ROS_INFO_STREAM("Parsed Move Group Goal:\n" << goal);

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
