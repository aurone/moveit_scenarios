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

#define OVERWRITE_FIELD(field) \
    do {\
        if (n[#field]) {\
            Overwrite(o.field, n[#field]);\
        }\
    } while(0)

bool Overwrite(std::uint8_t& b, YAML::Node n);
bool Overwrite(std::int8_t& b, YAML::Node n);
bool Overwrite(std::int32_t& i, YAML::Node n);
bool Overwrite(float& f, YAML::Node n);
bool Overwrite(double& d, YAML::Node n);
bool Overwrite(std::string& o, YAML::Node n);
bool Overwrite(std::uint32_t& o, YAML::Node n);
bool Overwrite(ros::Time& o, YAML::Node n);
bool Overwrite(std_msgs::Header& o, YAML::Node n);
bool Overwrite(std_msgs::ColorRGBA& o, YAML::Node n);
bool Overwrite(geometry_msgs::Vector3& o, YAML::Node n);
bool Overwrite(geometry_msgs::Quaternion& o, YAML::Node n);
bool Overwrite(geometry_msgs::Transform& o, YAML::Node n);
bool Overwrite(geometry_msgs::Twist& o, YAML::Node n);
bool Overwrite(geometry_msgs::Wrench& o, YAML::Node n);
bool Overwrite(geometry_msgs::Point& o, YAML::Node n);
bool Overwrite(geometry_msgs::Pose& o, YAML::Node n);
bool Overwrite(geometry_msgs::TransformStamped& o, YAML::Node n);
bool Overwrite(shape_msgs::SolidPrimitive& o, YAML::Node n);
bool Overwrite(shape_msgs::Mesh& o, YAML::Node n);
bool Overwrite(shape_msgs::MeshTriangle& o, YAML::Node n);
bool Overwrite(shape_msgs::Plane& o, YAML::Node n);
bool Overwrite(sensor_msgs::JointState& o, YAML::Node n);
bool Overwrite(sensor_msgs::MultiDOFJointState& o, YAML::Node n);
bool Overwrite(trajectory_msgs::JointTrajectory& o, YAML::Node n);
bool Overwrite(octomap_msgs::OctomapWithPose& o, YAML::Node n);
bool Overwrite(object_recognition_msgs::ObjectType& o, YAML::Node n);
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
bool Overwrite(moveit_msgs::AllowedCollisionEntry& o, YAML::Node n);
bool Overwrite(moveit_msgs::AllowedCollisionMatrix& o, YAML::Node n);
bool Overwrite(moveit_msgs::LinkPadding& o, YAML::Node n);
bool Overwrite(moveit_msgs::LinkScale& o, YAML::Node n);
bool Overwrite(moveit_msgs::ObjectColor& o, YAML::Node n);
bool Overwrite(moveit_msgs::PlanningSceneWorld& o, YAML::Node n);
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

template <typename T, std::size_t S>
bool Overwrite(boost::array<T, S>& o, YAML::Node n)
{
    int written = 0;
    for (auto it = n.begin(); it != n.end() && written < S; ++it) {
        YAML::Node nn = *it;
        Overwrite(o[written], nn);
        ++written;
    }
    return true;
}

//////////////////

bool Overwrite(std::uint8_t& b, YAML::Node n)
{
    if (n.Type() == YAML::NodeType::Null) {
        return false;
    }

    try {
        b = n.as<bool>(); // allow 'true' and 'false' boolean scalars
    }
    catch (...) {
        try {
            b = n.as<unsigned>();
        }
        catch (...) {
            return false;
        }
    }
    return true;
}

bool Overwrite(std::int8_t& b, YAML::Node n)
{
    b = (std::int8_t)n.as<int>();
    return true;
}

bool Overwrite(std::int32_t& i, YAML::Node n)
{
    if (n.Type() == YAML::NodeType::Null) {
        return false;
    }
    i = n.as<std::int32_t>();
    return true;
}

bool Overwrite(float& f, YAML::Node n)
{
    if (n.Type() == YAML::NodeType::Null) {
        return false;
    }
    f = n.as<float>();
    return true;
}

bool Overwrite(double& d, YAML::Node n)
{
    if (n.Type() == YAML::NodeType::Null) {
        return false;
    }
    d = n.as<double>();
    return true;
}

bool Overwrite(std::string& o, YAML::Node n)
{
    if (n.Type() == YAML::NodeType::Null) {
        return false;
    }
    o = n.as<std::string>();
    return true;
}

bool Overwrite(std::uint32_t& o, YAML::Node n)
{
    if (n.Type() == YAML::NodeType::Null) {
        return false;
    }
    o = n.as<std::uint32_t>();
    return true;
}

bool Overwrite(ros::Time& o, YAML::Node n)
{
    o = ros::Time(n.as<std::int32_t>() / 1000000000, n.as<std::int32_t>() % 1000000000);
}

bool Overwrite(geometry_msgs::Vector3& o, YAML::Node n)
{
    OVERWRITE_FIELD(x);
    OVERWRITE_FIELD(y);
    OVERWRITE_FIELD(z);
    return true;
}

bool Overwrite(geometry_msgs::Quaternion& o, YAML::Node n)
{
    OVERWRITE_FIELD(w);
    OVERWRITE_FIELD(x);
    OVERWRITE_FIELD(y);
    OVERWRITE_FIELD(z);
    return true;
}

bool Overwrite(geometry_msgs::Transform& o, YAML::Node n)
{
    OVERWRITE_FIELD(translation);
    OVERWRITE_FIELD(rotation);
    return true;
}

bool Overwrite(geometry_msgs::Twist& o, YAML::Node n)
{
    OVERWRITE_FIELD(linear);
    OVERWRITE_FIELD(angular);
    return true;
}

bool Overwrite(geometry_msgs::Wrench& o, YAML::Node n)
{
    OVERWRITE_FIELD(force);
    OVERWRITE_FIELD(torque);
    return true;
}

bool Overwrite(geometry_msgs::Point& o, YAML::Node n)
{
    OVERWRITE_FIELD(x);
    OVERWRITE_FIELD(y);
    OVERWRITE_FIELD(z);
    return true;
}

bool Overwrite(geometry_msgs::Pose& o, YAML::Node n)
{
    OVERWRITE_FIELD(position);
    OVERWRITE_FIELD(orientation);
    return true;
}

bool Overwrite(geometry_msgs::TransformStamped& o, YAML::Node n)
{
    OVERWRITE_FIELD(header);
    OVERWRITE_FIELD(child_frame_id);
    OVERWRITE_FIELD(transform);
    return true;
}

bool Overwrite(shape_msgs::SolidPrimitive& o, YAML::Node n)
{
    OVERWRITE_FIELD(type);
    OVERWRITE_FIELD(dimensions);
    return true;
}

bool Overwrite(shape_msgs::Mesh& o, YAML::Node n)
{
    OVERWRITE_FIELD(triangles);
    OVERWRITE_FIELD(vertices);
    return true;
}

bool Overwrite(shape_msgs::MeshTriangle& o, YAML::Node n)
{
    OVERWRITE_FIELD(vertex_indices);
    return true;
}

bool Overwrite(shape_msgs::Plane& o, YAML::Node n)
{
    OVERWRITE_FIELD(coef);
    return true;
}

bool Overwrite(std_msgs::Header& o, YAML::Node n)
{
    OVERWRITE_FIELD(seq);
    OVERWRITE_FIELD(stamp);
    OVERWRITE_FIELD(frame_id);
    return true;
}

bool Overwrite(std_msgs::ColorRGBA& o, YAML::Node n)
{
    OVERWRITE_FIELD(r);
    OVERWRITE_FIELD(g);
    OVERWRITE_FIELD(b);
    OVERWRITE_FIELD(a);
    return true;
}

bool Overwrite(sensor_msgs::JointState& o, YAML::Node n)
{
    OVERWRITE_FIELD(header);
    OVERWRITE_FIELD(name);
    OVERWRITE_FIELD(position);
    OVERWRITE_FIELD(velocity);
    OVERWRITE_FIELD(effort);
    return true;
}

bool Overwrite(sensor_msgs::MultiDOFJointState& o, YAML::Node n)
{
    OVERWRITE_FIELD(header);
    OVERWRITE_FIELD(joint_names);
    OVERWRITE_FIELD(transforms);
    OVERWRITE_FIELD(twist);
    OVERWRITE_FIELD(wrench);
    return true;
}

bool Overwrite(trajectory_msgs::JointTrajectory& o, YAML::Node n)
{
    ROS_ERROR_ONCE("%s unimplemented", __PRETTY_FUNCTION__);
    return true;
}

bool Overwrite(octomap_msgs::OctomapWithPose& o, YAML::Node n)
{
    ROS_ERROR_ONCE("%s unimplemented", __PRETTY_FUNCTION__);
    return true;
}

bool Overwrite(object_recognition_msgs::ObjectType& o, YAML::Node n)
{
    OVERWRITE_FIELD(key);
    OVERWRITE_FIELD(db);
    return true;
}

bool Overwrite(moveit_msgs::CollisionObject& o, YAML::Node n)
{
    OVERWRITE_FIELD(header);
    OVERWRITE_FIELD(id);
    OVERWRITE_FIELD(type);
    OVERWRITE_FIELD(primitives);
    OVERWRITE_FIELD(primitive_poses);
    OVERWRITE_FIELD(meshes);
    OVERWRITE_FIELD(mesh_poses);
    OVERWRITE_FIELD(planes);
    OVERWRITE_FIELD(plane_poses);
    OVERWRITE_FIELD(operation);
    return true;
}

bool Overwrite(moveit_msgs::AttachedCollisionObject& o, YAML::Node n)
{
    OVERWRITE_FIELD(link_name);
    OVERWRITE_FIELD(object);
    OVERWRITE_FIELD(touch_links);
    OVERWRITE_FIELD(detach_posture);
    OVERWRITE_FIELD(weight);
    return true;
}

bool Overwrite(moveit_msgs::WorkspaceParameters& o, YAML::Node n)
{
    OVERWRITE_FIELD(header);
    OVERWRITE_FIELD(min_corner);
    OVERWRITE_FIELD(max_corner);
    return true;
}

bool Overwrite(moveit_msgs::PlanningScene& o, YAML::Node n)
{
    OVERWRITE_FIELD(name);
    OVERWRITE_FIELD(robot_state);
    OVERWRITE_FIELD(robot_model_name);
    OVERWRITE_FIELD(fixed_frame_transforms);
    OVERWRITE_FIELD(allowed_collision_matrix);
    OVERWRITE_FIELD(link_padding);
    OVERWRITE_FIELD(link_scale);
    OVERWRITE_FIELD(object_colors);
    OVERWRITE_FIELD(world);
    OVERWRITE_FIELD(is_diff);
    return true;
}

bool Overwrite(moveit_msgs::RobotState& o, YAML::Node n)
{
    OVERWRITE_FIELD(joint_state);
    OVERWRITE_FIELD(multi_dof_joint_state);
    OVERWRITE_FIELD(attached_collision_objects);
    OVERWRITE_FIELD(is_diff);
    return true;
}

bool Overwrite(moveit_msgs::BoundingVolume& o, YAML::Node n)
{
    OVERWRITE_FIELD(primitives);
    OVERWRITE_FIELD(primitive_poses);
    OVERWRITE_FIELD(meshes);
    OVERWRITE_FIELD(mesh_poses);
    return true;
}

bool Overwrite(moveit_msgs::JointConstraint& o, YAML::Node n)
{
    OVERWRITE_FIELD(joint_name);
    OVERWRITE_FIELD(position);
    OVERWRITE_FIELD(tolerance_above);
    OVERWRITE_FIELD(tolerance_below);
    OVERWRITE_FIELD(weight);
    return true;
}

bool Overwrite(moveit_msgs::PositionConstraint& o, YAML::Node n)
{
    OVERWRITE_FIELD(header);
    OVERWRITE_FIELD(link_name);
    OVERWRITE_FIELD(target_point_offset);
    OVERWRITE_FIELD(constraint_region);
    OVERWRITE_FIELD(weight);
    return true;
}

bool Overwrite(moveit_msgs::OrientationConstraint& o, YAML::Node n)
{
    OVERWRITE_FIELD(header);
    OVERWRITE_FIELD(orientation);
    OVERWRITE_FIELD(link_name);
    OVERWRITE_FIELD(absolute_x_axis_tolerance);
    OVERWRITE_FIELD(absolute_y_axis_tolerance);
    OVERWRITE_FIELD(absolute_z_axis_tolerance);
    OVERWRITE_FIELD(weight);
    return true;
}

bool Overwrite(moveit_msgs::VisibilityConstraint& o, YAML::Node n)
{
    ROS_ERROR_ONCE("%s unimplemented", __PRETTY_FUNCTION__);
    return true;
}

bool Overwrite(moveit_msgs::Constraints& o, YAML::Node n)
{
    OVERWRITE_FIELD(name);
    OVERWRITE_FIELD(joint_constraints);
    OVERWRITE_FIELD(position_constraints);
    OVERWRITE_FIELD(orientation_constraints);
    OVERWRITE_FIELD(visibility_constraints);
    return true;
}

bool Overwrite(moveit_msgs::TrajectoryConstraints& o, YAML::Node n)
{
    OVERWRITE_FIELD(constraints);
    return true;
}

bool Overwrite(moveit_msgs::AllowedCollisionEntry& o, YAML::Node n)
{
    OVERWRITE_FIELD(enabled);
    return true;
}

bool Overwrite(moveit_msgs::AllowedCollisionMatrix& o, YAML::Node n)
{
    OVERWRITE_FIELD(entry_names);
    OVERWRITE_FIELD(entry_values);
    OVERWRITE_FIELD(default_entry_names);
    OVERWRITE_FIELD(default_entry_values);
    return true;
}

bool Overwrite(moveit_msgs::LinkPadding& o, YAML::Node n)
{
    OVERWRITE_FIELD(link_name);
    OVERWRITE_FIELD(padding);
    return true;
}

bool Overwrite(moveit_msgs::LinkScale& o, YAML::Node n)
{
    OVERWRITE_FIELD(link_name);
    OVERWRITE_FIELD(scale);
    return true;
}

bool Overwrite(moveit_msgs::ObjectColor& o, YAML::Node n)
{
    OVERWRITE_FIELD(id);
    OVERWRITE_FIELD(color);
    return true;
}

bool Overwrite(moveit_msgs::PlanningSceneWorld& o, YAML::Node n)
{
    OVERWRITE_FIELD(collision_objects);
    OVERWRITE_FIELD(octomap);
    return true;
}

bool Overwrite(moveit_msgs::MotionPlanRequest& o, YAML::Node n)
{
    OVERWRITE_FIELD(workspace_parameters);
    OVERWRITE_FIELD(start_state);
    OVERWRITE_FIELD(goal_constraints);
    OVERWRITE_FIELD(path_constraints);
    OVERWRITE_FIELD(trajectory_constraints);
    OVERWRITE_FIELD(planner_id);
    OVERWRITE_FIELD(group_name);
    OVERWRITE_FIELD(num_planning_attempts);
    OVERWRITE_FIELD(allowed_planning_time);
    OVERWRITE_FIELD(max_velocity_scaling_factor);
    OVERWRITE_FIELD(max_acceleration_scaling_factor);
    return true;
}

bool Overwrite(moveit_msgs::PlanningOptions& o, YAML::Node n)
{
    OVERWRITE_FIELD(planning_scene_diff);
    OVERWRITE_FIELD(plan_only);
    OVERWRITE_FIELD(look_around);
    OVERWRITE_FIELD(look_around_attempts);
    OVERWRITE_FIELD(max_safe_execution_cost);
    OVERWRITE_FIELD(replan);
    OVERWRITE_FIELD(replan_attempts);
    OVERWRITE_FIELD(replan_delay);
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

        for (auto it = root_scenario_path.begin();
                it != root_scenario_path.end(); ++it)
        {
            base_path_entries.push_back(*it);
        }

        for (auto it = scenario_path.begin(); it != scenario_path.end(); ++it) {
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

    ROS_DEBUG_STREAM("Parsed Move Group Goal:\n" << goal);

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
