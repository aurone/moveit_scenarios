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

    BoxSynchronizer();

    void sync(const moveit_msgs::CollisionObject& object);

private:

    ros::NodeHandle m_nh;
    ros::Publisher m_object_pub;
    ros::ServiceClient m_get_scene_srv;

    bool compareObjects(
        const moveit_msgs::CollisionObject& o1,
        const moveit_msgs::CollisionObject& o2) const;

    bool comparePrimitives(
        const moveit_msgs::CollisionObject& o1,
        const moveit_msgs::CollisionObject& o2) const;

    bool comparePrimitive(
        const shape_msgs::SolidPrimitive& p1,
        const shape_msgs::SolidPrimitive& p2) const;

    bool comparePlanes(
        const moveit_msgs::CollisionObject& o1,
        const moveit_msgs::CollisionObject& o2) const;

    bool comparePlane(
        const shape_msgs::Plane& p1,
        const shape_msgs::Plane& p2) const;

    bool comparePoses(
        const moveit_msgs::CollisionObject& o1,
        const moveit_msgs::CollisionObject& o2) const;

    // returns true if object was found in planning scene; false otherwise
    bool retrieveObject(const std::string& name, moveit_msgs::CollisionObject& obj);

    void addObject(const moveit_msgs::CollisionObject& o);

    void removeObject(const std::string& name);

    void moveObject(const moveit_msgs::CollisionObject& obj);
};

BoxSynchronizer::BoxSynchronizer() :
    m_nh(),
    m_object_pub(),
    m_get_scene_srv()
{
    ROS_INFO("Let ROS setup");
    m_object_pub = m_nh.advertise<moveit_msgs::CollisionObject>(
            "collision_object", 5);
    m_get_scene_srv = m_nh.serviceClient<moveit_msgs::GetPlanningScene>(
            "get_planning_scene");
    ros::Duration(1.0).sleep();
    ROS_INFO("...hope it's done");
}

/// Query the server for existence of a collision object. If the object does not
/// exist on the server, send a request to add the object. If the object does
/// exist but at a different pose, send a request to move the object. If an
/// object exists with the same name but is a different object, then send a
/// request to replace the object.
void BoxSynchronizer::sync(const moveit_msgs::CollisionObject& object)
{
    moveit_msgs::CollisionObject obj;
    try {
        if (retrieveObject(object.id, obj)) {
            if (!compareObjects(object, obj)) {
                ROS_INFO("objects differ");

                // may not be necessary if moveit automatically replaces old
                // objects
                removeObject(object.id);

                addObject(object);
            } else if (!comparePoses(object, obj)) {
                moveObject(object);
            }
        }
        else {
            addObject(object);
        }
    }
    catch (const std::runtime_error& ex) {
        // ...whatever, doing this on a loop now
    }
}

bool BoxSynchronizer::compareObjects(
    const moveit_msgs::CollisionObject& o1,
    const moveit_msgs::CollisionObject& o2) const
{
    if (o1.primitives.size() != o2.primitives.size() ||
        o1.planes.size() != o2.planes.size())
    {
        return false;
    }

    if (!o1.primitives.empty()) {
        return comparePrimitives(o1, o2);
    } else if (!o1.planes.empty()) {
        return comparePlanes(o1, o2);
    } else {
        // covers empty arrays (and meshes)
        return true;
    }
}

// Test if two objects are equal via the primitives they are composed of. The
// primitives must appear in the same order in both objects, have the same
// types, and have the same dimensions. Poses are not tested.
bool BoxSynchronizer::comparePrimitives(
    const moveit_msgs::CollisionObject& o1,
    const moveit_msgs::CollisionObject& o2) const
{
    if (o1.primitives.size() != o2.primitives.size()) {
        return false;
    }
    for (size_t i = 0; i < o1.primitives.size(); ++i) {
        if (!comparePrimitive(o1.primitives[i], o2.primitives[i])) {
            return false;
        }
    }
    return true;
}

// Test if two primitives have the same type and dimensions.
bool BoxSynchronizer::comparePrimitive(
    const shape_msgs::SolidPrimitive& p1,
    const shape_msgs::SolidPrimitive& p2) const
{
    if (p1.type != p2.type) {
        return false;
    }

    return p1.dimensions == p2.dimensions;
}

// Test if two planes are equal. The planes must appear in the same order in
// both objects and have the same dimensions. Poses are not tested.
bool BoxSynchronizer::comparePlanes(
    const moveit_msgs::CollisionObject& o1,
    const moveit_msgs::CollisionObject& o2) const
{
    if (o1.planes.size() != o2.planes.size()) {
        return false;
    }
    for (size_t i = 0; i < o1.planes.size(); ++i) {
        if (!comparePlane(o1.planes[i], o2.planes[i])) {
            return false;
        }
    }
    return true;
}

// Test if two planes have the same equation.
bool BoxSynchronizer::comparePlane(
    const shape_msgs::Plane& p1,
    const shape_msgs::Plane& p2) const
{
    return p1.coef == p2.coef;
}

bool BoxSynchronizer::comparePoses(
    const moveit_msgs::CollisionObject& o1,
    const moveit_msgs::CollisionObject& o2) const
{
    auto poses_equal = [](
        const geometry_msgs::Pose& p1,
        const geometry_msgs::Pose& p2)
    {
        return p1.position.x == p2.position.x &&
                p1.position.y == p2.position.y &&
                p1.position.z == p2.position.z &&
                p1.orientation.w == p2.orientation.w &&
                p1.orientation.x == p2.orientation.x &&
                p1.orientation.y == p2.orientation.y &&
                p1.orientation.z == p2.orientation.z;
    };

    // assumes the objects are the same
    for (size_t i = 0; i < o1.primitive_poses.size(); ++i) {
        const geometry_msgs::Pose& p1 = o1.primitive_poses[i];
        const geometry_msgs::Pose& p2 = o2.primitive_poses[i];
        if (!poses_equal(p1, p2)) {
            return false;
        }
    }
    for (size_t i = 0; i < o1.plane_poses.size(); ++i) {
        const geometry_msgs::Pose& p1 = o1.plane_poses[i];
        const geometry_msgs::Pose& p2 = o2.plane_poses[i];
        if (!poses_equal(p1, p2)) {
            return false;
        }
    }

    return true;
}

// returns true if object was found in planning scene; false otherwise
bool BoxSynchronizer::retrieveObject(
    const std::string& name,
    moveit_msgs::CollisionObject& obj)
{
    ROS_DEBUG("Waiting for /get_planning_scene service");
    m_get_scene_srv.waitForExistence();

    // get the planning scene
    moveit_msgs::GetPlanningScene packet;
    packet.request.components.components =
            moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
            moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    if (!m_get_scene_srv.call(packet)) {
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

void BoxSynchronizer::addObject(const moveit_msgs::CollisionObject& o)
{
    ROS_INFO("Add box '%s'", o.id.c_str());
    moveit_msgs::CollisionObject oo = o;
    oo.header.stamp = ros::Time::now();
    oo.operation = moveit_msgs::CollisionObject::ADD;
    m_object_pub.publish(o);
    ros::Duration(1.0).sleep();
}

void BoxSynchronizer::removeObject(const std::string& id)
{
    ROS_INFO("Remove box '%s'", id.c_str());
    moveit_msgs::CollisionObject oo;
    oo.header.stamp = ros::Time::now();
    oo.id = id;
    oo.operation = moveit_msgs::CollisionObject::REMOVE;
    m_object_pub.publish(oo);
    ros::Duration(1.0).sleep();
}

void BoxSynchronizer::moveObject(const moveit_msgs::CollisionObject& o)
{
    moveit_msgs::CollisionObject oo = o;
    oo.header.stamp = ros::Time::now();
    oo.operation = moveit_msgs::CollisionObject::MOVE;
    ROS_INFO("Moving oo '%s' in frame '%s'", oo.id.c_str(), oo.header.frame_id.c_str());
    m_object_pub.publish(oo);
    ros::Duration(1.0).sleep();
}

void PrintUsage()
{
    fprintf(stderr, "Usage: sync_box <type> <name> <x> <y> <z> <dims...>\n");
}

// Add a table to the planning scene if one does not exist or modify the table
// if the input arguments are different
int main(int argc, char* argv[])
{
    // this first to remove ros arguments
    ros::init(argc, argv, "sync_object", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    if (argc < 6) {
        PrintUsage();
        return 1;
    }

    std::string type_name(argv[1]);
    std::string object_name(argv[2]);

    // position args (even for planes)
    std::string xarg(argv[3]);
    std::string yarg(argv[4]);
    std::string zarg(argv[5]);
    double table_x = std::stod(xarg);
    double table_y = std::stod(yarg);
    double table_z = std::stod(zarg);

    moveit_msgs::CollisionObject object;
    object.header.frame_id = "map";
    object.id = object_name;

    if (type_name != "plane") {
        object.primitives.resize(1);
        object.primitive_poses.resize(1);
        object.primitive_poses.front().position.x = table_x;
        object.primitive_poses.front().position.y = table_y;
        object.primitive_poses.front().position.z = table_z;
        object.primitive_poses.front().orientation.w = 1.0;
    } else {
        object.planes.resize(1);
        object.plane_poses.resize(1);
        object.plane_poses.front().orientation.w = 1.0;
    }

    // fill in type-dependent variables
    if (type_name == "box") {
        if (argc < 9) {
            PrintUsage();
            return 1;
        }
        std::string length_arg(argv[6]);
        std::string width_arg(argv[7]);
        std::string height_arg(argv[8]);
        double length = std::stod(length_arg);
        double width = std::stod(width_arg);
        double height = std::stod(height_arg);

        shape_msgs::SolidPrimitive& prim = object.primitives.front();
        prim.type = shape_msgs::SolidPrimitive::BOX;
        prim.dimensions = { length, width, height };
    } else if (type_name == "sphere") {
        if (argc < 7) {
            PrintUsage();
            return 1;
        }
        std::string radius_arg(argv[6]);
        double radius = std::stod(radius_arg);

        shape_msgs::SolidPrimitive& prim = object.primitives.front();
        prim.type = shape_msgs::SolidPrimitive::SPHERE;
        prim.dimensions = { radius };
    } else if (type_name == "cylinder") {
        if (argc < 8) {
            PrintUsage();
            return 1;
        }
        std::string height_arg(argv[6]);
        std::string radius_arg(argv[7]);
        double height = std::stod(height_arg);
        double radius = std::stod(radius_arg);

        shape_msgs::SolidPrimitive& prim = object.primitives.front();
        prim.type = shape_msgs::SolidPrimitive::CYLINDER;
        prim.dimensions = { height, radius };
    } else if (type_name == "cone") {
        if (argc < 8) {
            PrintUsage();
            return 1;
        }
        std::string height_arg(argv[6]);
        std::string radius_arg(argv[7]);
        double height = std::stod(height_arg);
        double radius = std::stod(radius_arg);

        shape_msgs::SolidPrimitive& prim = object.primitives.front();
        prim.type = shape_msgs::SolidPrimitive::CONE;
        prim.dimensions = { height, radius };
    } else if (type_name == "plane") {
        if (argc < 10) {
            PrintUsage();
            return 1;
        }
        std::string a_arg(argv[6]);
        std::string b_arg(argv[7]);
        std::string c_arg(argv[8]);
        std::string d_arg(argv[9]);
        double a = std::stod(a_arg);
        double b = std::stod(b_arg);
        double c = std::stod(c_arg);
        double d = std::stod(d_arg);

        shape_msgs::Plane& plane = object.planes.front();
        plane.coef[0] = a;
        plane.coef[1] = b;
        plane.coef[2] = c;
        plane.coef[3] = d;
    }

    BoxSynchronizer synchronizer;

    ROS_INFO("Synchronizing collision object '%s'", object_name.c_str());
    ros::Rate sync_rate(1.0);
    while (ros::ok()) {
        synchronizer.sync(object);

        ros::spinOnce();
        sync_rate.sleep();
    }

    return 0;
}
