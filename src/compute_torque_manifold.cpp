#include <stdio.h>
#include <memory>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sbpl_manipulation_components/occupancy_grid.h>

namespace distance_field {
typedef boost::shared_ptr<PropagationDistanceField> PropagationDistanceFieldPtr;
}

class RightArmTorqueManifold
{
public:

    RightArmTorqueManifold() { }

    bool init(
        double size_x, double size_y, double size_z,
        double origin_x, double origin_y, double origin_z,
        double res,
        double weight);

    int run();

private:

    ros::NodeHandle m_nh;

    robot_model_loader::RobotModelLoaderPtr m_rm_loader;
    moveit::core::RobotModelPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    std::shared_ptr<sbpl_arm_planner::OccupancyGrid> m_df;

    double m_weight_lbs;

    ros::Publisher m_torque_manifold_pub;

    bool exportPoints(const std::vector<Eigen::Vector3d>& points) const;
};

bool RightArmTorqueManifold::init(
    double size_x, double size_y, double size_z,
    double origin_x, double origin_y, double origin_z,
    double res,
    double weight_lbs)
{
    // load the robot model and instantiate a robot state for it
    m_rm_loader.reset(new robot_model_loader::RobotModelLoader);
    m_robot_model = m_rm_loader->getModel();
    if (!m_robot_model) {
        ROS_ERROR("Failed to load robot model");
        return false;
    }
    m_robot_state.reset(new moveit::core::RobotState(m_robot_model));
    m_robot_state->setToDefaultValues();

    // instantiate a distance field in the "torso_lift_link" frame, for
    // visualization of the manifold
    double max_dist_m = 0.2;
    m_df.reset(new sbpl_arm_planner::OccupancyGrid(
            size_x, size_y, size_z, res, origin_x, origin_y, origin_z, max_dist_m));
    m_df->setReferenceFrame("torso_lift_link");

    // initialize end effector weight
    m_weight_lbs = weight_lbs;

    m_torque_manifold_pub = m_nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 5, true);

    return true;
}

int RightArmTorqueManifold::run()
{
    const Eigen::Affine3d& T_model_torso =
            m_robot_state->getFrameTransform("torso_lift_link");

    auto right_arm = m_robot_model->getJointModelGroup("right_arm");
    if (!right_arm) {
        ROS_ERROR("Failed to find joint model group 'right_arm'");
        return 1;
    }

    std::vector<Eigen::Vector3d> points;
    // loop over all cells in the distance field
    int gxc, gyc, gzc;
    m_df->getGridSize(gxc, gyc, gzc);

    int num_cells = gxc * gyc * gzc;
    int i = 0;
    for (int gx = 0; gx < gxc; ++gx) {
        for (int gy = 0; gy < gyc; ++gy) {
            for (int gz = 0; gz < gzc; ++gz) {
                if (i % (num_cells / 100) == 0) {
                    ROS_INFO("%d/%d (%0.3f%%): %zu points", i, num_cells, 100.0 * (double)i / num_cells, points.size());
                }
                i++;

                // get the world point in the torso_lift_link frame
                double wx, wy, wz;
                m_df->gridToWorld(gx, gy, gz, wx, wy, wz);

                // TODO: sample various orientations here? assume identity rot
                // in the torso_lift_link frame for now

                // transform this point into the model frame
                const Eigen::Affine3d T_model_point(
                        T_model_torso * Eigen::Translation3d(wx, wy, wz));

                // convert to pose...
                geometry_msgs::Pose pose_point_model;
                tf::poseEigenToMsg(T_model_point, pose_point_model);

                // run ik to this point to get a goal pose
                if (!m_robot_state->setFromIK(right_arm, pose_point_model)) {
//                    ROS_ERROR("Failed to compute IK");
                    continue;
                }

                // compute torques at this pose for the given weight
                Eigen::MatrixXd J = m_robot_state->getJacobian(right_arm, Eigen::Vector3d(0.17, 0.0, 0.0));

                double weight_lbs = m_weight_lbs;
                double kg_per_lb = 1.0 / 2.2;
                double gravity_z = -9.8;

                typedef Eigen::Matrix<double, 6, 1> Vector6d;
                Vector6d f;
                f(0) = 0.0;
                f(1) = 0.0;
                f(2) = gravity_z * weight_lbs * kg_per_lb;
                f(3) = 0.0;
                f(4) = 0.0;
                f(5) = 0.0;

                Eigen::VectorXd t = J.transpose() * f;

                // check against limits: if invalid, add this to the list of
                // voxels
                std::vector<double> rarm_torque_limits = {
                    30.0, 30.0, 30.0, 30.0, 30.0, 10.0, 10.0
                };

                for (int i = 0; i < 7; ++i) {
                    if (fabs(t(i)) > rarm_torque_limits[i]) {
//                        ROS_INFO("State %s infeasible due to torque limits", to_string(angles).c_str());
                        points.push_back(Eigen::Vector3d(wx, wy, wz));
                    }
                }
            }
        }
    }

    m_df->addPointsToField(points);

//    exportPoints(points);

    visualization_msgs::MarkerArray ma = m_df->getOccupiedVoxelsVisualization();
    for (auto& marker : ma.markers) {
        marker.ns = "torque_manifold";
    }
    m_torque_manifold_pub.publish(ma);

    // get visualization and publish
    ros::spin();
    return 0;
}

bool RightArmTorqueManifold::exportPoints(
    const std::vector<Eigen::Vector3d>& points) const
{
    FILE* f = fopen("torque_points.csv", "w");
    if (!f) {
        return false;
    }

    for (const auto& point : points) {
        fprintf(f, "%lf %lf %lf\n", point.x(), point.y(), point.z());
    }

    fclose(f);
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "compute_torque_manifold");
    ros::NodeHandle nh;

    const double ox = 0.0; //-0.4;
    const double oy = -1.2;
    const double oz = -2.0;
    const double size_x = 1.5 - ox;
    const double size_y = 1.2 - oy;
    const double size_z = 1.0 - oz;

    const double res = 0.10;
    const double weight_lbs = 10.0;

    RightArmTorqueManifold m;
    if (!m.init(size_x, size_y, size_z, ox, oy, oz, res, weight_lbs)) {
        ROS_ERROR("Failed to initialize Right Arm Torque Manifold");
        return 1;
    }

    return m.run();
}
