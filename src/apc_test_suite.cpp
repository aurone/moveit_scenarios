#include <fstream>
#include <sstream>
#include <string>

#include <sbpl_geometry_utils/utils.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

enum MainResult
{
    SUCCESS = 0,
    FAILED_TO_RETRIEVE_PARAMS,
    FAILED_TO_LOAD_POSES,
    INSUFFICIENT_POSES,
    FAILED_TO_MOVE_TO_LOCALIZATION_POSE,
    FAILED_TO_TRANSFORM_COMMAND
};

bool LoadPoses(
    const std::string& poses_fname,
    std::vector<geometry_msgs::Pose>& poses)
{
    std::ifstream ifs(poses_fname);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open '%s' for reading", poses_fname.c_str());
        return false;
    }

    double x, y, z, qx, qy, qz, qw;
    std::string line;
    while (ifs.good()) {
        std::getline(ifs, line);
        std::stringstream ss(line);
        ss >> x >> y >> z >> qx >> qy >> qz >> qw;
        if (!ss.fail()) {
            geometry_msgs::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.x = qx;
            pose.orientation.y = qy;
            pose.orientation.z = qz;
            pose.orientation.w = qw;
            poses.push_back(pose);
        }
    }

    ROS_INFO("Loaded %zu poses from %s", poses.size(), poses_fname.c_str());
    return true;
}

class ApcTestSuite
{
public:

    ApcTestSuite() :
        m_ph("~"),
        m_spinner(1),
        m_listener(new tf::TransformListener)
    {
        m_spinner.start();
        m_listener.reset(new tf::TransformListener);
    }

    MainResult run()
    {
        MainResult res = initializeGoalPoses();
        if (res != MainResult::SUCCESS) {
            return res;
        }

        initializeMoveGroup();
        res = moveThroughLocalizationGoals();
        if (res != MainResult::SUCCESS) {
            return res;
        }
        moveThroughPickingGoals();
        ROS_INFO("Finished testing!!!");
        ROS_INFO("Test Results:");
        auto ll_count = std::count(m_localization_results.begin(), m_localization_results.end(), true);
        ROS_INFO("  Localization Success: %zd/%zu (%0.3f%%)", ll_count, m_localization_results.size(), 100.0 * (double)ll_count / m_localization_results.size());
        auto ia_count = std::count(m_image_acquisition_results.begin(), m_image_acquisition_results.end(), true);
        ROS_INFO("  Image Acquisition Success: %zd/%zu (%0.3f%%)", ia_count, m_image_acquisition_results.size(), 100.0 * (double)ia_count / m_image_acquisition_results.size());
        auto pp_count = std::count(m_preposition_results.begin(), m_preposition_results.end(), true);
        ROS_INFO("  Preposition Success: %zd/%zu (%0.3f%%)", pp_count, m_preposition_results.size(), 100.0 * (double)pp_count / m_preposition_results.size());
        auto pg_count = std::count(m_pregrasp_results.begin(), m_pregrasp_results.end(), true);
        ROS_INFO("  Pregrasp Success: %zd/%zu (%0.3f%%)", pg_count, m_pregrasp_results.size(), 100.0 * (double)pg_count / m_pregrasp_results.size());
        auto wd_count = std::count(m_withdraw_results.begin(), m_withdraw_results.end(), true);
        ROS_INFO("  Withdraw Success: %zd/%zu (%0.3f%%)", wd_count, m_withdraw_results.size(), 100.0 * (double)wd_count / m_withdraw_results.size());
    }

private:

    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;
    ros::AsyncSpinner m_spinner;

    std::unique_ptr<tf::TransformListener> m_listener;

    std::vector<geometry_msgs::Pose> m_dropoff_poses;
    std::vector<geometry_msgs::Pose> m_image_acquisition_poses;
    std::vector<geometry_msgs::Pose> m_localization_poses;
    std::vector<geometry_msgs::Pose> m_preposition_poses;
    std::vector<geometry_msgs::Pose> m_withdraw_poses;

    std::unique_ptr<move_group_interface::MoveGroup> m_move_group;

    std::vector<bool> m_localization_results;
    std::vector<bool> m_image_acquisition_results;
    std::vector<bool> m_preposition_results;
    std::vector<bool> m_pregrasp_results;
    std::vector<bool> m_withdraw_results;

    MainResult initializeGoalPoses()
    {
        std::string dropoff_poses_fname;
        std::string image_acquisition_poses_fname;
        std::string localization_poses_fname;
        std::string preposition_poses_fname;
        std::string withdraw_poses_fname;

        if (!m_ph.getParam("dropoff_poses_filename", dropoff_poses_fname) ||
            !m_ph.getParam("image_acquisition_poses_filename", image_acquisition_poses_fname) ||
            !m_ph.getParam("localization_poses_filename", localization_poses_fname) ||
            !m_ph.getParam("preposition_poses_filename", preposition_poses_fname) ||
            !m_ph.getParam("withdraw_poses_filename", withdraw_poses_fname))
        {
            ROS_ERROR("Failed to retrieve poses filenames from the param server");
            return MainResult::FAILED_TO_RETRIEVE_PARAMS;
        }

        if (!LoadPoses(dropoff_poses_fname, m_dropoff_poses) ||
            !LoadPoses(image_acquisition_poses_fname, m_image_acquisition_poses) ||
            !LoadPoses(localization_poses_fname, m_localization_poses) ||
            !LoadPoses(preposition_poses_fname, m_preposition_poses) ||
            !LoadPoses(withdraw_poses_fname, m_withdraw_poses))
        {
            ROS_ERROR("Failed to load poses from file");
            return MainResult::FAILED_TO_LOAD_POSES;
        }

        if (m_image_acquisition_poses.size() != m_preposition_poses.size() ||
            m_image_acquisition_poses.size() != m_withdraw_poses.size())
        {
            ROS_ERROR("image acquisition, preposition, and withdraw pose counts must match");
            return MainResult::INSUFFICIENT_POSES;
        }

        if (m_localization_poses.empty()) {
            ROS_ERROR("there must be at least one localization pose");
            return MainResult::INSUFFICIENT_POSES;
        }

        m_localization_results.assign(m_localization_poses.size(), false);
        m_image_acquisition_results.assign(m_image_acquisition_poses.size(), false);
        m_preposition_results.assign(m_preposition_poses.size(), false);
        m_pregrasp_results.assign(m_preposition_poses.size(), false);
        m_withdraw_results.assign(m_withdraw_poses.size(), false);

        return MainResult::SUCCESS;
    }

    void initializeMoveGroup()
    {
        m_move_group.reset(new move_group_interface::MoveGroup("manipulator"));

        boost::shared_ptr<tf::Transformer> tf(new tf::TransformListener);
        while (!tf->waitForTransform("base_link", m_move_group->getPlanningFrame(), ros::Time::now(), ros::Duration(1.0))) {
            ROS_WARN("Waiting for transform (%s, base_link)", m_move_group->getPlanningFrame().c_str());
        }

        m_move_group->setGoalJointTolerance(0.0);
        m_move_group->setGoalOrientationTolerance(sbpl::utils::ToRadians(1.0));
        m_move_group->setGoalPositionTolerance(0.05);
        m_move_group->setNumPlanningAttempts(1);
        m_move_group->setPlannerId("LARA*");
        m_move_group->setPlanningTime(15.0);
        m_move_group->setStartStateToCurrentState();
        double min_x = -0.40, min_y = -1.20, min_z = -1.36;
        double max_x =  1.50, max_y =  1.20, max_z =  1.00;
        tf::Stamped<tf::Point> workspace_min_base_link(tf::Point(min_x, min_y, min_z), ros::Time::now(), "base_link");
        tf::Stamped<tf::Point> workspace_max_base_link(tf::Point(max_x, max_y, max_z), ros::Time::now(), "base_link");
        tf::Stamped<tf::Point> workspace_min_planning_frame;
        tf::Stamped<tf::Point> workspace_max_planning_frame;
        tf->transformPoint(m_move_group->getPlanningFrame(), workspace_min_base_link, workspace_min_planning_frame);
        tf->transformPoint(m_move_group->getPlanningFrame(), workspace_max_base_link, workspace_max_planning_frame);

        m_move_group->setWorkspace(
                workspace_min_planning_frame.x(),
                workspace_min_planning_frame.y(),
                workspace_min_planning_frame.z(),
                workspace_max_planning_frame.x(),
                workspace_max_planning_frame.y(),
                workspace_max_planning_frame.z());

        m_move_group->allowLooking(false);
        m_move_group->allowReplanning(false);
    }

    MainResult moveThroughLocalizationGoals()
    {
        for (size_t i = 0; i < m_localization_poses.size(); ++i) {
            const auto& localization_pose = m_localization_poses[i];
            geometry_msgs::Pose eet_pose;
            while (!getEndEffectorTargetPose(localization_pose, eet_pose)) {
                ROS_WARN("Failed to transform MAS command pose to target ee pose");
            }
            m_move_group->setPoseTarget(eet_pose, "ee_link");

            moveit::planning_interface::MoveGroup::Plan plan;
            moveit::planning_interface::MoveItErrorCode err;

            ROS_INFO_STREAM("Moving to ee pose " << eet_pose);
            err = m_move_group->move();

            m_localization_results[i] = (err.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
            if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_ERROR("Failed to move to target ee pose");
            }
        }

        return MainResult::SUCCESS;
    }

    MainResult moveThroughPickingGoals()
    {
        for (size_t i = 0; i < m_image_acquisition_poses.size(); ++i) {
            geometry_msgs::Pose eet_pose;
            moveit::planning_interface::MoveItErrorCode err;

            const auto& image_acquisition_pose = m_image_acquisition_poses[i];
            const auto& preposition_pose = m_preposition_poses[i];
            geometry_msgs::Pose pregrasp_pose = m_preposition_poses[i];
            pregrasp_pose.position.x += 0.30;
            const auto& withdraw_pose = m_withdraw_poses[i];

            // move to the image acquisition pose
            while (!getEndEffectorTargetPose(image_acquisition_pose, eet_pose)) {
                ROS_WARN("Failed to transform MAS command pose to target ee pose");
            }

            m_move_group->setPoseTarget(eet_pose, "ee_link");
            err = m_move_group->move();

            m_image_acquisition_results[i] = (err.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
            if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_ERROR("Failed to move to target image acquisition pose");
            }

            // move to the preposition pose
            while (!getEndEffectorTargetPose(preposition_pose, eet_pose)) {
                ROS_WARN("Failed to transform MAS command pose to target ee pose");
            }

            m_move_group->setPoseTarget(eet_pose, "ee_link");
            err = m_move_group->move();

            m_preposition_results[i] = (err.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
            if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_ERROR("Failed to move to target preposition pose");
            }

            // move to the pregrasp pose
            while (!getEndEffectorTargetPose(pregrasp_pose, eet_pose)) {
                ROS_WARN("Failed to transform MAS command pose to target ee pose");
            }

            m_move_group->setPoseTarget(eet_pose, "ee_link");
            err = m_move_group->move();

            m_pregrasp_results[i] = (err.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
            if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_ERROR("Failed to move to target pregrasp pose");
            }

            // move to the with draw pose
            while (!getEndEffectorTargetPose(withdraw_pose, eet_pose)) {
                ROS_WARN("Failed to transform MAS command pose to target withdraw pose");
            }

            m_move_group->setPoseTarget(eet_pose, "ee_link");
            err = m_move_group->move();

            m_withdraw_results[i] = (err.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
            if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_ERROR("Failed to move to target withdraw pose");
            }
        }
    }

    bool getEndEffectorTargetPose(
        const geometry_msgs::Pose& MAS_command_pose,
        geometry_msgs::Pose& eet_pose)
    {
        // Holy mother of God...

        // update the transform from the map to the command pose
        ros::param::set("/command_pos_x", MAS_command_pose.position.x);
        ros::param::set("/command_pos_y", MAS_command_pose.position.y);
        ros::param::set("/command_pos_z", MAS_command_pose.position.z);
        ros::param::set("/command_rot_x", MAS_command_pose.orientation.x);
        ros::param::set("/command_rot_y", MAS_command_pose.orientation.y);
        ros::param::set("/command_rot_z", MAS_command_pose.orientation.z);
        ros::param::set("/command_rot_w", MAS_command_pose.orientation.w);

        // wait a bit for dynamic_tf_publisher to grab the new pose and broadcast
        // the transform
        ros::Duration(0.3).sleep();

        // Pray
        try {
            tf::StampedTransform ee_tf;
            m_listener->lookupTransform("map", "eet_desired", ros::Time(0), ee_tf);
            eet_pose.position.x = ee_tf.getOrigin()[0];
            eet_pose.position.y = ee_tf.getOrigin()[1];
            eet_pose.position.z = ee_tf.getOrigin()[2];
            eet_pose.orientation.x = ee_tf.getRotation().x();
            eet_pose.orientation.y = ee_tf.getRotation().y();
            eet_pose.orientation.z = ee_tf.getRotation().z();
            eet_pose.orientation.w = ee_tf.getRotation().w();
            return true;
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "apc_test_suite");
    return ApcTestSuite().run();
    ros::shutdown();
    return SUCCESS;
}
