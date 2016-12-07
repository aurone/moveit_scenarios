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

#ifndef BirdhouseTestSuite_h
#define BirdhouseTestSuite_h

#include <math.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <leatherman/viz.h>
#include <leatherman/print.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

std::vector<geometry_msgs::Pose> SampleBirdhouseGoalPoses(
    const geometry_msgs::Vector3& sizes,
    double padding,
    int levels = 1);

bool ExportGoals(const std::vector<geometry_msgs::Pose>& poses);

class BirdhouseTestSuite
{
public:

    BirdhouseTestSuite();

    bool init();
    int run();

private:

    /// \name General to both test sequences
    /// @{
    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;

    robot_model_loader::RobotModelLoaderPtr m_rm_loader;
    moveit::core::RobotModelPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    ros::ServiceClient m_plan_path_client;
    ros::ServiceClient m_execute_path_client;
    ros::Publisher m_goal_marker_pub;
    ///@}

    bool m_precached_goals;

    /// \name Generate goals sequence
    ///@{
    geometry_msgs::Vector3 m_birdhouse_size;
    geometry_msgs::Pose m_birdhouse_pose;
    double m_birdhouse_padding;
    std::vector<geometry_msgs::Pose> m_wrist_goals;
    std::vector<std::vector<double>> m_valid_goal_states;

    std::vector<double> m_right_arm_initial_state;
    ///@}

    /// \name Precached goals sequence
    ///@{
    std::string m_goals_filename;
    std::vector<std::vector<double>> m_goal_joint_states;
    double m_success_planning_time_total;

    struct TestResult
    {
        std::vector<double> start;
        std::vector<double> goal;
        bool success;
        double planning_time;
    };

    std::vector<TestResult> m_results;
    ///@}

    bool initGeneral();
    bool initPrecachedGoalsSequence();
    bool initGenerateGoalsSequence();

    int runGeneral();
    int runPrecachedGoalsSequence();
    int runGenerateGoalsSequence();

    bool moveLeftArm();

    bool planToGoal(const geometry_msgs::Pose& goal_pose);

    bool exportValidGoals();

    bool planBetweenPoints(
        const std::vector<double>& start_state,
        const std::vector<double>& goal_state);

    void fillWorkspaceParameters(
        moveit_msgs::MotionPlanRequest& req,
        const ros::Time& when) const;

    bool exportResults() const;
};

#endif
