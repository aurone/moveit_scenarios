#include "birdhouse_test_suite.h"

#include <fstream>

// Sample a number of poses along the frame of the birdhouse. The positions lie
// on the surface of the bounding box with orientations pointing inward along
// the surface normal.
std::vector<geometry_msgs::Pose> SampleBirdhouseGoalPoses(
    const geometry_msgs::Vector3& sizes,
    double padding)
{
    double front_x = 0.5 * sizes.x;
    double back_x = -0.5 * sizes.x;
    double right_y = 0.5 * sizes.y;
    double left_y = -0.5 * sizes.y;
    double top_z = 0.5 * sizes.z;
    double bottom_z = -0.5 * sizes.z;

    Eigen::Vector3d front_inward_normal(-1.0, 0.0, 0.0);
    Eigen::Vector3d right_inward_normal(0.0, -1.0, 0.0);
    Eigen::Vector3d top_inward_normal(0.0, 0.0, -1.0);

    auto compute_rotation = [&](
        const Eigen::Vector3d& n,
        const Eigen::Vector3d& up) -> Eigen::Quaterniond
    {
        // NOTE: n and up assumed to be normalized
        Eigen::Vector3d left = n.cross(up);
        left.normalize();
        Eigen::Vector3d right = -left;
        Eigen::Vector3d new_up = n.cross(right);
        Eigen::Matrix3d R;
        R(0, 0) = n(0);
        R(1, 0) = n(1);
        R(2, 0) = n(2);
        R(0, 1) = right(0);
        R(1, 1) = right(1);
        R(2, 1) = right(2);
        R(0, 2) = new_up(0);
        R(1, 2) = new_up(1);
        R(2, 2) = new_up(2);
        return Eigen::Quaterniond(R);
    };

    std::vector<geometry_msgs::Pose> poses;

    geometry_msgs::Pose p;
    Eigen::Quaterniond q;

    //////////////////////////////////////
    // compute poses for the front face //
    //////////////////////////////////////

    q = compute_rotation(front_inward_normal, Eigen::Vector3d(0.0, 0.0, 1.0));
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();

    p.position.x = front_x + padding;
    p.position.y = right_y;
    p.position.z = top_z;
    poses.push_back(p);

    p.position.x = front_x + padding;
    p.position.y = right_y;
    p.position.z = bottom_z;
    poses.push_back(p);

    p.position.x = front_x + padding;
    p.position.y = left_y;
    p.position.z = bottom_z;
    poses.push_back(p);

    p.position.x = front_x + padding;
    p.position.y = left_y;
    p.position.z = top_z;
    poses.push_back(p);

    p.position.x = front_x + padding;
    p.position.y = 0.0;
    p.position.z = 0.0;
    poses.push_back(p);

    //////////////////////////////////////
    // compute poses for the right face //
    //////////////////////////////////////

    q = compute_rotation(right_inward_normal, Eigen::Vector3d(0.0, 0.0, 1.0));
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();

    p.position.y = right_y + padding;
    p.position.x = front_x;
    p.position.z = top_z;
    poses.push_back(p);

    p.position.y = right_y + padding;
    p.position.x = front_x;
    p.position.z = bottom_z;
    poses.push_back(p);

    p.position.y = right_y + padding;
    p.position.x = back_x;
    p.position.z = bottom_z;
    poses.push_back(p);

    p.position.y = right_y + padding;
    p.position.x = back_x;
    p.position.z = top_z;
    poses.push_back(p);

    p.position.y = right_y + padding;
    p.position.x = 0.0;
    p.position.z = 0.0;
    poses.push_back(p);

    ////////////////////////////////////
    // compute poses for the top face //
    ////////////////////////////////////

    q = compute_rotation(top_inward_normal, Eigen::Vector3d(-0.7071, -0.7071, 0.0));
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();

    p.position.z = top_z + padding;
    p.position.x = front_x;
    p.position.y = right_y;
    poses.push_back(p);

    p.position.z = top_z + padding;
    p.position.x = front_x;
    p.position.y = left_y;
    poses.push_back(p);

    p.position.z = top_z + padding;
    p.position.x = back_x;
    p.position.y = left_y;
    poses.push_back(p);

    p.position.z = top_z + padding;
    p.position.x = back_x;
    p.position.y = right_y;
    poses.push_back(p);

    p.position.z = top_z + padding;
    p.position.x = 0.0;
    p.position.y = 0.0;
    poses.push_back(p);

    return poses;
}

bool ExportGoals(const std::vector<geometry_msgs::Pose>& goals)
{
    FILE* f = fopen("goals.csv", "w");
    if (!f) {
        return false;
    }

    for (size_t i = 0; i < goals.size(); ++i) {
        double x = goals[i].position.x;
        double y = goals[i].position.y;
        double z = goals[i].position.z;
        double qw = goals[i].orientation.w;
        double qx = goals[i].orientation.x;
        double qy = goals[i].orientation.y;
        double qz = goals[i].orientation.z;
        fprintf(f, "%lf %lf %lf %lf %lf %lf %lf\n", x, y, z, qw, qx, qy, qz);
    }

    fclose(f);
    return true;
}

BirdhouseTestSuite::BirdhouseTestSuite() :
    m_nh(),
    m_ph("~"),
    m_rm_loader(),
    m_robot_model(),
    m_robot_state(),
    m_birdhouse_pose(),
    m_success_planning_time_total(0.0)
{
    m_plan_path_client = m_nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
    m_execute_path_client = m_nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
    m_goal_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);
}

bool BirdhouseTestSuite::init()
{
    bool precached_goals;
    if (!m_ph.getParam("precached_goals", precached_goals)) {
        ROS_ERROR("Failed to retrieve 'precached_goals' from the param server");
        return false;
    }

    m_precached_goals = precached_goals;

    if (!initGeneral()) {
        ROS_ERROR("Failed to initialize common data");
        return false;
    }

    if (m_precached_goals) {
        return initPrecachedGoalsSequence();
    }
    else {
        return initGenerateGoalsSequence();
    }
}

int BirdhouseTestSuite::run()
{
    int res = runGeneral();
    if (res != 0) {
        return res;
    }

    if (m_precached_goals) {
        return runPrecachedGoalsSequence();
    }
    else {
        return runGenerateGoalsSequence();
    }
}

bool BirdhouseTestSuite::initGeneral()
{
    /////////////////////
    // construct robot //
    /////////////////////

    m_rm_loader.reset(new robot_model_loader::RobotModelLoader);
    m_robot_model = m_rm_loader->getModel();
    if (!m_robot_model) {
        ROS_ERROR("Failed to load robot model");
        return false;
    }

    m_robot_state.reset(new moveit::core::RobotState(m_robot_model));
    m_robot_state->setToDefaultValues();
    m_robot_state->setVariablePosition("l_shoulder_pan_joint", M_PI / 2.0);
    m_robot_state->updateLinkTransforms();

    return true;
}

bool BirdhouseTestSuite::initPrecachedGoalsSequence()
{
    std::string goals_filename;
    if (!m_ph.getParam("goals_filename", goals_filename)) {
        ROS_ERROR("Failed to retrieve 'goals_filename' from the param server");
        return false;
    }

    std::ifstream ifs(goals_filename);

    double j1, j2, j3, j4, j5, j6, j7;
    while (ifs >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> j7) {
        std::vector<double> j = { j1, j2, j3, j4, j5, j6, j7 };
        m_goal_joint_states.push_back(std::move(j));
    }

    ROS_INFO("Loaded %zu joint states", m_goal_joint_states.size());

    return true;
}

bool BirdhouseTestSuite::initGenerateGoalsSequence()
{
    std::string init_state_str;
    if (!m_ph.getParam("right_arm_initial_state", init_state_str)) {
        ROS_ERROR("Failed to retrieve 'right_arm_initial_state' from the param server");
        return false;
    }

    std::stringstream ss;
    ss << init_state_str;

    std::vector<double> joint_variable_values;
    double d;
    while (ss >> d) {
        joint_variable_values.push_back(sbpl::utils::ToRadians(d));
    }

    if (joint_variable_values.size() != 7) {
        ROS_ERROR("Insufficient initial joint variable values (expected: 7, actual: %zu)", joint_variable_values.size());
        return false;
    }

    m_right_arm_initial_state = std::move(joint_variable_values);

    //////////////////////////
    // initialize birdhouse //
    //////////////////////////

    double birdhouse_x;
    double birdhouse_y;
    double birdhouse_z;
    double birdhouse_length;
    double birdhouse_width;
    double birdhouse_height;
    double birdhouse_roll;
    double birdhouse_pitch;
    double birdhouse_yaw;
    double birdhouse_padding;

    if (!m_ph.getParam("birdhouse_x", birdhouse_x) ||
        !m_ph.getParam("birdhouse_y", birdhouse_y) ||
        !m_ph.getParam("birdhouse_z", birdhouse_z) ||
        !m_ph.getParam("birdhouse_roll", birdhouse_roll) ||
        !m_ph.getParam("birdhouse_pitch", birdhouse_pitch) ||
        !m_ph.getParam("birdhouse_yaw", birdhouse_yaw) ||
        !m_ph.getParam("birdhouse_length", birdhouse_length) ||
        !m_ph.getParam("birdhouse_width", birdhouse_width) ||
        !m_ph.getParam("birdhouse_height", birdhouse_height) ||
        !m_ph.getParam("birdhouse_padding", birdhouse_padding))
    {
        ROS_ERROR("Failed to retrieve birdhouse parameters from the param server");
        return false;
    }

    Eigen::Affine3d T_model_box =
        Eigen::Translation3d(birdhouse_x, birdhouse_y, birdhouse_z) *
        Eigen::AngleAxisd(birdhouse_yaw, Eigen::Vector3d(0.0, 0.0, 1.0)) *
        Eigen::AngleAxisd(birdhouse_pitch, Eigen::Vector3d(0.0, 1.0, 0.0)) *
        Eigen::AngleAxisd(birdhouse_roll, Eigen::Vector3d(1.0, 0.0, 0.0));

    tf::poseEigenToMsg(T_model_box, m_birdhouse_pose);

    m_birdhouse_size.x = birdhouse_length;
    m_birdhouse_size.y = birdhouse_width;
    m_birdhouse_size.z = birdhouse_height;

    m_birdhouse_padding = birdhouse_padding;

    ////////////////////////////////////////
    // sample goals relative to birdhouse //
    ////////////////////////////////////////

    std::vector<geometry_msgs::Pose> bh_samples;
    bh_samples = SampleBirdhouseGoalPoses(m_birdhouse_size, m_birdhouse_padding);

    ROS_INFO("Birdhouse goals:");
    for (size_t i = 0; i < bh_samples.size(); ++i) {
        ROS_INFO_STREAM("  " << i << ": " << bh_samples[i]);
    }

    ////////////////////////////////////////////////////////////////////
    // transform goals in box frame to goals for wrist in model frame //
    ////////////////////////////////////////////////////////////////////

    Eigen::Affine3d T_model_wrist = m_robot_state->getFrameTransform("r_wrist_roll_link");
    Eigen::Affine3d T_model_tool = m_robot_state->getFrameTransform("r_gripper_tool_frame");
    Eigen::Affine3d T_wrist_tool = T_model_wrist.inverse() * T_model_tool;

    std::vector<geometry_msgs::Pose> wrist_goals;
    for (size_t i = 0; i < bh_samples.size(); ++i) {
        const geometry_msgs::Pose& sample_pose = bh_samples[i];
        Eigen::Affine3d T_box_sample;
        tf::poseMsgToEigen(sample_pose, T_box_sample);

        // the sample pose is where we would like to place the tool frame
        Eigen::Affine3d T_box_tool_goal = T_box_sample;

        Eigen::Affine3d T_model_wrist_goal =
                T_model_box * T_box_tool_goal * T_wrist_tool.inverse();

        geometry_msgs::Pose wrist_goal;
        tf::poseEigenToMsg(T_model_wrist_goal, wrist_goal);

        wrist_goals.push_back(wrist_goal);
    }

    m_wrist_goals = std::move(wrist_goals);
    return true;
}

int BirdhouseTestSuite::runGeneral()
{
    return 0;
}

int BirdhouseTestSuite::runPrecachedGoalsSequence()
{
    int succ_count = 0;
    int num_trials = 0;
    for (size_t i = 0; i < m_goal_joint_states.size(); ++i) {
        for (size_t j = 0; j < m_goal_joint_states.size(); ++j) {
            if (i == j) {
                continue;
            }

            ++num_trials;

            const std::vector<double>& start_state = m_goal_joint_states[i];
            const std::vector<double>& goal_state = m_goal_joint_states[j];

            if (planBetweenPoints(start_state, goal_state)) {
                ++succ_count;
            }
        }
    }

    ROS_WARN("%d/%d succeeded in %0.3f seconds (%0.3f average)",
            succ_count,
            num_trials,
            m_success_planning_time_total,
            succ_count == 0 ? 0.0 : m_success_planning_time_total / succ_count);
    return 0;
}

int BirdhouseTestSuite::runGenerateGoalsSequence()
{
    if (!moveLeftArm()) {
        ROS_ERROR("Failed to move left arm out of the way");
        return 1;
    }

    m_robot_state->setVariablePosition("r_shoulder_pan_joint", m_right_arm_initial_state[0]);
    m_robot_state->setVariablePosition("r_shoulder_lift_joint", m_right_arm_initial_state[1]);
    m_robot_state->setVariablePosition("r_upper_arm_roll_joint", m_right_arm_initial_state[2]);
    m_robot_state->setVariablePosition("r_elbow_flex_joint", m_right_arm_initial_state[3]);
    m_robot_state->setVariablePosition("r_forearm_roll_joint", m_right_arm_initial_state[4]);
    m_robot_state->setVariablePosition("r_wrist_flex_joint", m_right_arm_initial_state[5]);
    m_robot_state->setVariablePosition("r_wrist_roll_joint", m_right_arm_initial_state[6]);

    size_t success_count = 0;
    for (size_t i = 0; i < m_wrist_goals.size(); ++i) {
        ROS_INFO("Sending goal %zu/%zu", i, m_wrist_goals.size());
        const geometry_msgs::Pose& wrist_goal = m_wrist_goals[i];
        ROS_INFO_STREAM(wrist_goal);
        m_goal_marker_pub.publish(viz::getPoseMarkerArray(wrist_goal, m_robot_model->getModelFrame(), "goal"));
        ros::spinOnce();
        if (planToGoal(wrist_goal)) {
            ++success_count;
        }
    }

    ROS_WARN("%zu/%zu succeeded", success_count, m_wrist_goals.size());
    if (!exportValidGoals()) {
        ROS_ERROR("Failed to export valid goals");
        return 1;
    }

    return 0;
}

bool BirdhouseTestSuite::moveLeftArm()
{
    ROS_INFO("Moving left arm");
    moveit_msgs::ExecuteKnownTrajectory srv;

    auto left_arm = m_robot_model->getJointModelGroup("left_arm");
    if (!left_arm) {
        ROS_ERROR("Fuck");
        return false;
    }

    std::vector<std::string> var_names(left_arm->getVariableCount());
    std::vector<double> start(left_arm->getVariableCount());
    std::vector<double> goal(left_arm->getVariableCount());
    std::vector<double> min_limits(left_arm->getVariableCount());
    std::vector<double> max_limits(left_arm->getVariableCount());
    std::vector<double> inc(left_arm->getVariableCount(), sbpl::utils::ToRadians(1.0));
    std::vector<bool> continuous(left_arm->getVariableCount());

    for (size_t i = 0; i < left_arm->getVariableCount(); ++i) {
        const std::string& var_name = left_arm->getVariableNames()[i];
        var_names[i] = var_name;
        start[i] = m_robot_state->getVariablePosition(var_name);
        goal[i] = m_robot_state->getVariablePosition(var_name);

        auto jm = m_robot_model->getJointOfVariable(var_name);
        auto& var_bounds = jm->getVariableBounds(var_name);
        if (var_bounds.position_bounded_) {
            min_limits[i] = var_bounds.min_position_;
            max_limits[i] = var_bounds.max_position_;
            continuous[i] = true;
        }
        else {
            min_limits[i] = -M_PI;
            max_limits[i] = M_PI;
            continuous[i] = false;
        }
    }

    goal[0] = M_PI / 2.0; // NOTE: embedded shoulder-is-first-joint-knowledge

    std::vector<std::vector<double>> path;
    if (!sbpl::interp::InterpolatePath(
            start, goal, min_limits, max_limits, inc, continuous, path))
    {
        ROS_ERROR("Failed to produce move left arm trajectory");
        return false;
    }

    trajectory_msgs::JointTrajectory& traj = srv.request.trajectory.joint_trajectory;
    traj.header.seq = 0;
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = m_robot_model->getModelFrame();

    traj.joint_names = var_names;

    ros::Duration time_from_start(0.0);

    for (const auto& point : path) {
        trajectory_msgs::JointTrajectoryPoint traj_pt;

        traj_pt.positions = point;
        traj_pt.velocities.clear();
        traj_pt.accelerations.clear();
        traj_pt.effort.clear();
        traj_pt.time_from_start = time_from_start;

        traj.points.push_back(traj_pt);

        time_from_start += ros::Duration(0.1);
    }

    srv.request.wait_for_execution = true;

    if (!m_execute_path_client.call(srv)) {
        ROS_ERROR("Failed to call execute path client");
        return false;
    }

    return true;
}

bool BirdhouseTestSuite::planToGoal(const geometry_msgs::Pose& goal_pose)
{
    moveit_msgs::GetMotionPlan srv;
    moveit_msgs::MotionPlanRequest& req = srv.request.motion_plan_request;
    const ros::Time now = ros::Time::now();

    //////////////////////////
    // workspace_parameters //
    //////////////////////////

    fillWorkspaceParameters(req, now);

    /////////////////
    // start_state //
    /////////////////

    moveit::core::robotStateToRobotStateMsg(*m_robot_state, req.start_state);

    //////////////////////
    // goal_constraints //
    //////////////////////

    const std::string& tip_link = "r_wrist_roll_link";

    moveit_msgs::Constraints goal_constraints;
    goal_constraints.name = "goal_constraints";

    // Position constraint on the tip link

    moveit_msgs::PositionConstraint goal_pos_constraint;

    goal_pos_constraint.header.frame_id = m_robot_model->getModelFrame();
    goal_pos_constraint.header.seq = 0;
    goal_pos_constraint.header.stamp = now;

    goal_pos_constraint.link_name = tip_link;

    goal_pos_constraint.target_point_offset.x = 0.0;
    goal_pos_constraint.target_point_offset.y = 0.0;
    goal_pos_constraint.target_point_offset.z = 0.0;

    // specify region within 5cm of the goal position
    shape_msgs::SolidPrimitive tolerance_volume;
    tolerance_volume.type = shape_msgs::SolidPrimitive::SPHERE;
    tolerance_volume.dimensions = { 0.05 };
    goal_pos_constraint.constraint_region.primitives.push_back(tolerance_volume);
    goal_pos_constraint.constraint_region.primitive_poses.push_back(goal_pose);

    goal_pos_constraint.weight = 1.0;

    // Orientation constraint on the tip link

    // specify goal orientation within 5 degrees of the goal orientation
    moveit_msgs::OrientationConstraint goal_rot_constraint;

    goal_rot_constraint.header.frame_id = m_robot_model->getModelFrame();
    goal_rot_constraint.header.seq = 0;
    goal_rot_constraint.header.stamp = now;

    goal_rot_constraint.orientation = goal_pose.orientation;

    goal_rot_constraint.link_name = tip_link;

    goal_rot_constraint.absolute_x_axis_tolerance = sbpl::utils::ToRadians(10.0);
    goal_rot_constraint.absolute_y_axis_tolerance = sbpl::utils::ToRadians(10.0);
    goal_rot_constraint.absolute_z_axis_tolerance = sbpl::utils::ToRadians(10.0);

    goal_rot_constraint.weight = 1.0;

//    goal_constraints.joint_constraints;
    goal_constraints.position_constraints.push_back(goal_pos_constraint);
    goal_constraints.orientation_constraints.push_back(goal_rot_constraint);
//    goal_constraints.visibility_constraints;

    req.goal_constraints.push_back(goal_constraints);

    //////////////
    // the rest //
    //////////////

    req.planner_id = "ARA*";
    req.group_name = "right_arm";
    req.num_planning_attempts = 1;
    req.allowed_planning_time = 10.0;
    req.max_velocity_scaling_factor = 1.0;

    //////////
    // plan //
    //////////

    if (!m_plan_path_client.call(srv)) {
        ROS_ERROR("Failed to call 'plan_kinematic_path' service");
        return false;
    }

    moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;
    auto jmg = m_robot_model->getJointModelGroup("right_arm");
    if (!jmg) {
        ROS_ERROR("What the fuck?");
        return false;
    }

    if (response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        std::vector<double> valid_goal;
        for (const std::string& joint_name : jmg->getActiveJointModelNames()) {
            double pos;
            auto it = std::find(
                response.trajectory.joint_trajectory.joint_names.begin(),
                response.trajectory.joint_trajectory.joint_names.end(),
                joint_name);
            if (it == response.trajectory.joint_trajectory.joint_names.end()) {
                ROS_ERROR("What the fuck?? no %s in %s", joint_name.c_str(), to_string(response.trajectory.joint_trajectory.joint_names).c_str());
                return false;
            }

            size_t jidx = std::distance(response.trajectory.joint_trajectory.joint_names.begin(), it);
            trajectory_msgs::JointTrajectoryPoint& final_point = response.trajectory.joint_trajectory.points.back();

            valid_goal.push_back(final_point.positions[jidx]);
        }

        m_valid_goal_states.push_back(valid_goal);
    }

    return true;
}

bool BirdhouseTestSuite::exportValidGoals()
{
    FILE* f = fopen("precached_goals.csv", "w");
    if (!f) {
        return false;
    }

    for (const auto& goal : m_valid_goal_states) {
        assert(goal.size() == 7);
        fprintf(f, "%lf %lf %lf %lf %lf %lf %lf\n", goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], goal[6]);
    }

    fclose(f);
    return true;
}

bool BirdhouseTestSuite::planBetweenPoints(
    const std::vector<double>& start_state,
    const std::vector<double>& goal_state)
{
    assert(start_state.size() == 7 && goal_state.size() == 7);

    moveit_msgs::GetMotionPlan srv;
    moveit_msgs::MotionPlanRequest& req = srv.request.motion_plan_request;
    const ros::Time now = ros::Time::now();

    //////////////////////////
    // workspace_parameters //
    //////////////////////////

    fillWorkspaceParameters(req, now);

    /////////////////
    // start_state //
    /////////////////

    moveit::core::RobotState start_robot_state = *m_robot_state;
    start_robot_state.setVariablePosition("r_shoulder_pan_joint", start_state[0]);
    start_robot_state.setVariablePosition("r_shoulder_lift_joint", start_state[1]);
    start_robot_state.setVariablePosition("r_upper_arm_roll_joint", start_state[2]);
    start_robot_state.setVariablePosition("r_elbow_flex_joint", start_state[3]);
    start_robot_state.setVariablePosition("r_forearm_roll_joint", start_state[4]);
    start_robot_state.setVariablePosition("r_wrist_flex_joint", start_state[5]);
    start_robot_state.setVariablePosition("r_wrist_roll_joint", start_state[6]);
    moveit::core::robotStateToRobotStateMsg(start_robot_state, req.start_state);

    //////////////////////
    // goal_constraints //
    //////////////////////

    moveit::core::RobotState goal_robot_state = *m_robot_state;
    goal_robot_state.setVariablePosition("r_shoulder_pan_joint", goal_state[0]);
    goal_robot_state.setVariablePosition("r_shoulder_lift_joint", goal_state[1]);
    goal_robot_state.setVariablePosition("r_upper_arm_roll_joint", goal_state[2]);
    goal_robot_state.setVariablePosition("r_elbow_flex_joint", goal_state[3]);
    goal_robot_state.setVariablePosition("r_forearm_roll_joint", goal_state[4]);
    goal_robot_state.setVariablePosition("r_wrist_flex_joint", goal_state[5]);
    goal_robot_state.setVariablePosition("r_wrist_roll_joint", start_state[6]);
    goal_robot_state.updateLinkTransforms();

    const std::string& tip_link = "r_wrist_roll_link";

    const Eigen::Affine3d& T_model_goal_wrist = goal_robot_state.getGlobalLinkTransform(tip_link);
    geometry_msgs::Pose goal_pose;
    tf::poseEigenToMsg(T_model_goal_wrist, goal_pose);

    m_goal_marker_pub.publish(viz::getPoseMarkerArray(goal_pose, m_robot_model->getModelFrame(), "goal"));
    ros::spinOnce();

    moveit_msgs::Constraints goal_constraints;
    goal_constraints.name = "goal_constraints";

    // Position constraint on the tip link

    moveit_msgs::PositionConstraint goal_pos_constraint;

    goal_pos_constraint.header.frame_id = m_robot_model->getModelFrame();
    goal_pos_constraint.header.seq = 0;
    goal_pos_constraint.header.stamp = now;

    goal_pos_constraint.link_name = tip_link;

    goal_pos_constraint.target_point_offset.x = 0.0;
    goal_pos_constraint.target_point_offset.y = 0.0;
    goal_pos_constraint.target_point_offset.z = 0.0;

    // specify region within 5cm of the goal position
    shape_msgs::SolidPrimitive tolerance_volume;
    tolerance_volume.type = shape_msgs::SolidPrimitive::SPHERE;
    tolerance_volume.dimensions = { 0.05 };
    goal_pos_constraint.constraint_region.primitives.push_back(tolerance_volume);
    goal_pos_constraint.constraint_region.primitive_poses.push_back(goal_pose);

    goal_pos_constraint.weight = 1.0;

    // Orientation constraint on the tip link

    // specify goal orientation within 5 degrees of the goal orientation
    moveit_msgs::OrientationConstraint goal_rot_constraint;

    goal_rot_constraint.header.frame_id = m_robot_model->getModelFrame();
    goal_rot_constraint.header.seq = 0;
    goal_rot_constraint.header.stamp = now;

    goal_rot_constraint.orientation = goal_pose.orientation;

    goal_rot_constraint.link_name = tip_link;

    goal_rot_constraint.absolute_x_axis_tolerance = sbpl::utils::ToRadians(10.0);
    goal_rot_constraint.absolute_y_axis_tolerance = sbpl::utils::ToRadians(10.0);
    goal_rot_constraint.absolute_z_axis_tolerance = sbpl::utils::ToRadians(10.0);

    goal_rot_constraint.weight = 1.0;

//    goal_constraints.joint_constraints;
    goal_constraints.position_constraints.push_back(goal_pos_constraint);
    goal_constraints.orientation_constraints.push_back(goal_rot_constraint);
//    goal_constraints.visibility_constraints;

    req.goal_constraints.push_back(goal_constraints);

    //////////////
    // the rest //
    //////////////

    req.planner_id = "ARA*";
    req.group_name = "right_arm";
    req.num_planning_attempts = 1;
    req.allowed_planning_time = 10.0;
    req.max_velocity_scaling_factor = 1.0;

    //////////
    // plan //
    //////////

    if (!m_plan_path_client.call(srv)) {
        ROS_ERROR("Failed to call 'plan_kinematic_path' service");
        return false;
    }

    const moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;

    if (response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        m_success_planning_time_total += response.planning_time;
        ROS_INFO("Plan path succeeded in %0.3f seconds", response.planning_time);
        return true;
    }

    ROS_ERROR("Plan path failed");
    return false;
}

void BirdhouseTestSuite::fillWorkspaceParameters(
    moveit_msgs::MotionPlanRequest& req,
    const ros::Time& when) const
{
    req.workspace_parameters.header.frame_id = "torso_lift_link";
    req.workspace_parameters.header.seq = 0;
    req.workspace_parameters.header.stamp = when;
    req.workspace_parameters.min_corner.x = -0.4;
    req.workspace_parameters.min_corner.y = -1.2;
    req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = 1.5;
    req.workspace_parameters.max_corner.y = 1.2;
    req.workspace_parameters.max_corner.z = 1.0;
}
