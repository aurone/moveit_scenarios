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
#include <algorithm>
#include <string>
#include <stdexcept>

// system includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometric_shapes/mesh_operations.h>

// class for ensuring that a given mesh exists in the planning scene
class MeshSynchronizer
{
public:

    MeshSynchronizer() :
        m_nh(),
        m_collision_object_pub(),
        m_get_planning_scene_srv()
    {
        ROS_INFO("Hating everything");
        m_collision_object_pub =
                m_nh.advertise<moveit_msgs::CollisionObject>(
                        "collision_object", 5);
        m_get_planning_scene_srv =
                m_nh.serviceClient<moveit_msgs::GetPlanningScene>(
                        "get_planning_scene");
        ros::Duration(1.0).sleep();
        ROS_INFO("...still hate everything");
    }

    bool init(
        const std::string& name,
        const std::string& fname,
        const geometry_msgs::PoseStamped& pose,
        double scale = 1.0)
    {
        shapes::Mesh* mesh = shapes::createMeshFromResource(fname);
        if (!mesh) {
            ROS_ERROR("Failed to create mesh from resource '%s'", fname.c_str());
            return false;
        }

        m_name = name;
        m_fname = fname;
        m_mesh = mesh;
        m_frame = pose.header.frame_id;
        m_pose = pose.pose;
        m_scale = scale;
        m_mesh_sig = getMeshSignature();
        return true;
    }

    void sync()
    {
        if (m_name.empty()) {
            return;
        }

        moveit_msgs::CollisionObject obj;
        try {
            if (retrieveCollisionObject(obj)) {
                if (sameCollisionObject(obj) && samePosition(obj)) {
                    // box is the same box
                    ROS_DEBUG("Mesh is the same mesh");
                    return;
                }
                else if (sameCollisionObject(obj) && !samePosition(obj)) {
                    // box moved position
                    ROS_INFO("Pose is different");
                    moveMesh(obj);
                }
                else if (!sameCollisionObject(obj) && samePosition(obj)) {
                    // mesh diff? TODO: check for scale here or potentially
                    // entire mesh
                    ROS_INFO("Mesh is different");
                    removeObject();
                    addMesh();
                }
                else {
                    // box is completely different
                    ROS_INFO("Everything is different");
                    removeObject();
                    addMesh();
                }
            }
            else {
                // there is no box
                ROS_INFO("There is no spoon...");
                addMesh();
            }
        }
        catch (const std::runtime_error& ex) {
            // ...whatever, doing this on a loop now
        }
    }

private:

    ros::NodeHandle m_nh;
    ros::Publisher m_collision_object_pub;
    ros::ServiceClient m_get_planning_scene_srv;

    std::string m_name;
    std::string m_fname;
    shapes::Mesh* m_mesh;
    double m_mesh_sig;

    std::string m_frame;
    geometry_msgs::Pose m_pose;

    double m_scale;

    double getMeshSignature() const
    {
        double sig = 0;
        for (int i = 0; i < m_mesh->vertex_count; ++i) {
            double ix = m_scale * m_mesh->vertices[3 * i + 0];
            double iy = m_scale * m_mesh->vertices[3 * i + 1];
            double iz = m_scale * m_mesh->vertices[3 * i + 2];
            for (int j = i + 1; j < m_mesh->vertex_count; ++j) {
                double jx = m_scale * m_mesh->vertices[3 * j + 0];
                double jy = m_scale * m_mesh->vertices[3 * j + 1];
                double jz = m_scale * m_mesh->vertices[3 * j + 2];

                double dsq = (jx - ix) * (jx - ix) + (jy - iy) * (jy - iy) + (jz - iz) * (jz - iz);
                sig += dsq;
            }
        }
        return sig;
    }

    double getMeshSignature(const shape_msgs::Mesh& mesh) const
    {
        double sig = 0;
        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            double ix = mesh.vertices[i].x;
            double iy = mesh.vertices[i].y;
            double iz = mesh.vertices[i].z;
            for (size_t j = i + 1; j < mesh.vertices.size(); ++j) {
                double jx = mesh.vertices[j].x;
                double jy = mesh.vertices[j].y;
                double jz = mesh.vertices[j].z;

                double dsq = (jx - ix) * (jx - ix) + (jy - iy) * (jy - iy) + (jz - iz) * (jz - iz);
                sig += dsq;
            }
        }
        return sig;
    }

    bool sameCollisionObject(const moveit_msgs::CollisionObject& obj) const
    {
        return obj.meshes.size() == 1 &&
            obj.meshes.front().vertices.size() == m_mesh->vertex_count &&
            obj.meshes.front().triangles.size() == m_mesh->triangle_count &&
            fabs(getMeshSignature(obj.meshes.front()) - m_mesh_sig) < 1e-3;
    }

    bool samePosition(const moveit_msgs::CollisionObject& obj) const
    {
        return obj.mesh_poses.size() == 1 &&
                obj.mesh_poses.front().orientation.w == m_pose.orientation.w &&
                obj.mesh_poses.front().orientation.x == m_pose.orientation.x &&
                obj.mesh_poses.front().orientation.y == m_pose.orientation.y &&
                obj.mesh_poses.front().orientation.z == m_pose.orientation.z &&
                obj.mesh_poses.front().position.x == m_pose.position.x &&
                obj.mesh_poses.front().position.y == m_pose.position.y &&
                obj.mesh_poses.front().position.z == m_pose.position.z;
   }

    // returns true if box was found in planning scene; false otherwise
    bool retrieveCollisionObject(moveit_msgs::CollisionObject& obj)
    {
        ROS_DEBUG("Waiting for /get_planning_scene service");
        m_get_planning_scene_srv.waitForExistence();

        // get the planning scene
        moveit_msgs::GetPlanningScene packet;
        packet.request.components.components =
                moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
        if (!m_get_planning_scene_srv.call(packet)) {
            ROS_ERROR("Failed to call service");
            throw std::runtime_error("Failed to retrieve planning scene");
        }

        // look for the mesh
        const moveit_msgs::PlanningSceneWorld& world =
                packet.response.scene.world;
        auto it = std::find_if(
            world.collision_objects.begin(), world.collision_objects.end(),
            [&](const moveit_msgs::CollisionObject& o)
            {
                return o.id == m_name;
            });

        if (it != world.collision_objects.end()) {
            obj = *it;
            return true;
        }
        else {
            return false;
        }
    }

    void addMesh()
    {
        moveit_msgs::CollisionObject mesh_object;
        mesh_object.header.seq = 0;
        mesh_object.header.stamp = ros::Time::now();
        mesh_object.header.frame_id = m_frame;
        mesh_object.id = m_name;

        shape_msgs::Mesh mesh;

        // convert shapes::Mesh to shape_msgs::Mesh
        mesh.vertices.resize(m_mesh->vertex_count);
        for (int i = 0; i < m_mesh->vertex_count; ++i) {
            mesh.vertices[i].x = m_scale * m_mesh->vertices[3 * i + 0];
            mesh.vertices[i].y = m_scale * m_mesh->vertices[3 * i + 1];
            mesh.vertices[i].z = m_scale * m_mesh->vertices[3 * i + 2];
        }

        mesh.triangles.resize(m_mesh->triangle_count);
        for (int i = 0; i < m_mesh->triangle_count; ++i) {
            mesh.triangles[i].vertex_indices[0] = m_mesh->triangles[3 * i + 0];
            mesh.triangles[i].vertex_indices[1] = m_mesh->triangles[3 * i + 1];
            mesh.triangles[i].vertex_indices[2] = m_mesh->triangles[3 * i + 2];
        }

        geometry_msgs::Pose mesh_pose = m_pose;

        mesh_object.meshes.push_back(mesh);
        mesh_object.mesh_poses.push_back(mesh_pose);

        mesh_object.operation = moveit_msgs::CollisionObject::ADD;

        ROS_INFO("Adding mesh '%s'", m_name.c_str());
        m_collision_object_pub.publish(mesh_object);
        ros::Duration(1.0).sleep();
    }

    void removeObject()
    {
        moveit_msgs::CollisionObject box_object;
        box_object.header.stamp = ros::Time::now();
        box_object.header.frame_id = m_frame;
        box_object.id = m_name;
        box_object.operation = moveit_msgs::CollisionObject::REMOVE;
        ROS_INFO("Removing mesh '%s'", m_name.c_str());
        m_collision_object_pub.publish(box_object);
        ros::Duration(1.0).sleep();
    }

    void moveMesh(const moveit_msgs::CollisionObject& obj)
    {
        moveit_msgs::CollisionObject box = obj;
        box.header.seq = 0;
        box.header.stamp = ros::Time::now();
        box.header.frame_id = m_frame;
        box.mesh_poses.front() = m_pose;
        box.meshes.clear();
        box.operation = moveit_msgs::CollisionObject::MOVE;
        ROS_INFO("Moving mesh '%s' in frame '%s'", box.id.c_str(), box.header.frame_id.c_str());
        m_collision_object_pub.publish(box);
        ros::Duration(1.0).sleep();
    }
};

// Add a table to the planning scene if one does not exist or modify the table
// if the input arguments are different
int main(int argc, char* argv[])
{
    // x, y, z, length, width, height
    if (argc < 11) {
        fprintf(stderr, "Usage: sync_mesh <name> <resource> <x> <y> <z> <r> <p> <y> <scale> <frame>\n");
        return 1;
    }

    for (int i = 0; i < argc; ++i) {
        printf("%s ", argv[i]);
    }
    printf("\n");

    std::string mesh_name(argv[1]);
    std::string mesh_resource(argv[2]);
    double pos_x =      std::stod(argv[3]);
    double pos_y =      std::stod(argv[4]);
    double pos_z =      std::stod(argv[5]);
    double pose_roll =  std::stod(argv[6]);
    double pose_pitch = std::stod(argv[7]);
    double pose_yaw =   std::stod(argv[8]);
    double scale =      std::stod(argv[9]);
    std::string frame = argv[10];

    Eigen::Affine3d transform;
    transform = Eigen::Translation3d(pos_x, pos_y, pos_z) *
        Eigen::AngleAxisd(pose_yaw, Eigen::Vector3d(0.0, 0.0, 1.0)) *
        Eigen::AngleAxisd(pose_pitch, Eigen::Vector3d(0.0, 1.0, 0.0)) *
        Eigen::AngleAxisd(pose_roll, Eigen::Vector3d(1.0, 0.0, 0.0));

    ros::init(argc, argv, "add_table");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;
    tf::poseEigenToMsg(transform, pose.pose);

    MeshSynchronizer synchronizer;
    if (!synchronizer.init(mesh_name, mesh_resource, pose, scale)) {
        ROS_ERROR("Failed to initialize mesh synchronizer");
        return 1;
    }

    ros::Rate loop_rate(10.0);
    const ros::Rate sync_rate(1.0);
    ros::Time last_sync = ros::Time(0);

    ROS_INFO("Synchronizing mesh object '%s'", mesh_name.c_str());
    while (ros::ok()) {
        const ros::Time now = ros::Time::now();
        if (last_sync < (now - sync_rate.expectedCycleTime())) {
            ROS_DEBUG("last sync: %0.3f", last_sync.toSec());
            ROS_DEBUG("now: %0.3f", now.toSec());
            ROS_DEBUG("sync cycle: %0.3f", sync_rate.expectedCycleTime().toSec());
            synchronizer.sync();
            last_sync = now;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
