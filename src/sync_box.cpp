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

#include <algorithm>
#include <string>
#include <stdexcept>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPlanningScene.h>

// class for ensuring that a given box exists in the planning scene
class BoxSynchronizer
{
public:

    BoxSynchronizer() :
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

    void sync(
        const std::string& name,
        double x, double y, double z,
        double length, double width, double height)
    {
        moveit_msgs::CollisionObject obj;
        try {
            if (retrieveBox(name, obj)) {
                if (obj.primitives.size() != 1 ||
                    obj.primitives.front().type != shape_msgs::SolidPrimitive::BOX)
                {
                    // this is not the box you're looking for
                    removeObject(name);
                    addBox(name, x, y, z, length, width, height);
                }
                else if (hasDimensions(obj, length, width, height) &&
                    hasPosition(obj, x, y, z))
                {
                    // box is the same box
                    ROS_DEBUG("Box is the same box");
                    return;
                }
                else if (hasDimensions(obj, length, width, height)) {
                    // box moved position
                    moveBox(obj, x, y, z);
                }
                else if (hasPosition(obj, x, y, z)) {
                    // box grew or shrunk
                    removeObject(name);
                    addBox(name, x, y, z, length, width, height);
                }
                else {
                    // box is completely different
                    removeObject(name);
                    addBox(name, x, y, z, length, width, height);
                }
            }
            else {
                // there is no box
                addBox(name, x, y, z, length, width, height);
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

    bool hasDimensions(
        const moveit_msgs::CollisionObject& obj,
        double length,
        double width,
        double height) const
    {
        return obj.primitives.size() == 1 &&
                obj.primitives.front().dimensions.size() == 3 &&
                obj.primitives.front().dimensions[0] == length &&
                obj.primitives.front().dimensions[1] == width &&
                obj.primitives.front().dimensions[2] == height;
    }

    bool hasPosition(
        const moveit_msgs::CollisionObject& obj,
        double x, double y, double z) const
    {
        return obj.primitive_poses.size() == 1 &&
                obj.primitive_poses.front().orientation.w == 1.0 &&
                obj.primitive_poses.front().orientation.x == 0.0 &&
                obj.primitive_poses.front().orientation.y == 0.0 &&
                obj.primitive_poses.front().orientation.z == 0.0 &&
                obj.primitive_poses.front().position.x == x &&
                obj.primitive_poses.front().position.y == y &&
                obj.primitive_poses.front().position.z == z;
   }

    // returns true if box was found in planning scene; false otherwise
    bool retrieveBox(const std::string& name, moveit_msgs::CollisionObject& obj)
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

        // look for the box
        const moveit_msgs::PlanningSceneWorld& world =
                packet.response.scene.world;
        auto it = std::find_if(
            world.collision_objects.begin(), world.collision_objects.end(),
            [&](const moveit_msgs::CollisionObject& o)
            {
                return o.id == name;
            });

        if (it != world.collision_objects.end()) {
            obj = *it;
            return true;
        }
        else {
            return false;
        }
    }

    void addBox(
        const std::string& name,
        double x, double y, double z,
        double length, double width, double height)
    {
        moveit_msgs::CollisionObject box_object;
        box_object.header.seq = 0;
        box_object.header.stamp = ros::Time::now();
        box_object.header.frame_id = "odom_combined";
        box_object.id = name;
        shape_msgs::SolidPrimitive primitive;
        primitive.type = shape_msgs::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = length;
        primitive.dimensions[1] = width;
        primitive.dimensions[2] = height;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.orientation.x = box_pose.orientation.y = box_pose.orientation.z = 0.0;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;

        box_object.primitives.push_back(primitive);
        box_object.primitive_poses.push_back(box_pose);

        box_object.operation = moveit_msgs::CollisionObject::ADD;

        ROS_INFO("Adding box '%s'", name.c_str());
        m_collision_object_pub.publish(box_object);
        ros::Duration(1.0).sleep();
    }

    void removeObject(const std::string& name)
    {
        moveit_msgs::CollisionObject box_object;
        box_object.header.stamp = ros::Time::now();
        box_object.id = name;
        box_object.operation = moveit_msgs::CollisionObject::REMOVE;
        ROS_INFO("Removing box '%s'", name.c_str());
        m_collision_object_pub.publish(box_object);
        ros::Duration(1.0).sleep();
    }

    void moveBox(
        const moveit_msgs::CollisionObject& obj,
        double x, double y, double z)
    {
        moveit_msgs::CollisionObject box = obj;
        box.header.seq = 0;
        box.header.stamp = ros::Time::now();
        box.primitive_poses.front().position.x = x;
        box.primitive_poses.front().position.y = y;
        box.primitive_poses.front().position.z = z;
        box.primitives.clear();
        box.operation = moveit_msgs::CollisionObject::MOVE;
        ROS_INFO("Moving box '%s' in frame '%s'", box.id.c_str(), box.header.frame_id.c_str());
        m_collision_object_pub.publish(box);
        ros::Duration(1.0).sleep();
    }
};

// Add a table to the planning scene if one does not exist or modify the table
// if the input arguments are different
int main(int argc, char* argv[])
{
    // x, y, z, length, width, height
    if (argc < 8) {
        fprintf(stderr, "Usage: sync_box <x> <y> <z> <length> <width> <height> <name>\n");
        return 1;
    }

    std::string xarg(argv[1]);
    std::string yarg(argv[2]);
    std::string zarg(argv[3]);
    std::string length_arg(argv[4]);
    std::string width_arg(argv[5]);
    std::string height_arg(argv[6]);
    std::string box_name(argv[7]);

    double table_x = std::stod(xarg);
    double table_y = std::stod(yarg);
    double table_z = std::stod(zarg);
    double table_length = std::stod(length_arg);
    double table_width = std::stod(width_arg);
    double table_height = std::stod(height_arg);

    ros::init(argc, argv, "add_table");
    ros::NodeHandle nh;

    BoxSynchronizer synchronizer;

    ros::Rate loop_rate(10.0);
    const ros::Rate sync_rate(1.0);
    ros::Time last_sync = ros::Time(0);

    ROS_INFO("Synchronizing box object '%s'", box_name.c_str());
    while (ros::ok()) {
        const ros::Time now = ros::Time::now();
        if (last_sync < (now - sync_rate.expectedCycleTime())) {
            ROS_DEBUG("last sync: %0.3f", last_sync.toSec());
            ROS_DEBUG("now: %0.3f", now.toSec());
            ROS_DEBUG("sync cycle: %0.3f", sync_rate.expectedCycleTime().toSec());
            // synchronize box
            synchronizer.sync(
                    box_name,
                    table_x, table_y, table_z,
                    table_length, table_width, table_height);
            last_sync = now;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
