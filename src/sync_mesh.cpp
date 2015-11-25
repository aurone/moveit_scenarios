#include <algorithm>
#include <string>
#include <stdexcept>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometric_shapes/mesh_operations.h>

// class for ensuring that a given box exists in the planning scene
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

    bool init(const std::string& name, const std::string& fname, const geometry_msgs::Point& pos)
    {
        shapes::Mesh* mesh = shapes::createMeshFromResource(fname);
        if (!mesh) {
            ROS_ERROR("Failed to create mesh from resource '%s'", fname.c_str());
            return false;
        }

        m_name = name;
        m_fname = fname;
        m_mesh = mesh;
        m_pos = pos;
        return true;
    }

    void sync()
    {
        if (m_name.empty()) {
            return;
        }

        moveit_msgs::CollisionObject obj;
        try {
            if (retrieveMesh(obj)) {
                if (obj.meshes.size() != 1) {
                    // this is not the box you're looking for
                    removeObject();
                    addMesh();
                }
                else if (true /* TODO: same mesh at some pose */) {
                    // box is the same box
                    ROS_DEBUG("Mesh is the same mesh");
                    return;
                }
                else if (false /* TODO: same mesh at different pose */) {
                    // box moved position
                    moveBox(obj);
                }
                else if (false /* TODO: different mesh at same pose */) {
                    // box grew or shrunk
                    removeObject();
                    addMesh();
                }
                else {
                    // box is completely different
                    removeObject();
                    addMesh();
                }
            }
            else {
                // there is no box
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

    geometry_msgs::Point m_pos;

    bool hasPosition(const moveit_msgs::CollisionObject& obj) const
    {
        return obj.mesh_poses.size() == 1 &&
                obj.mesh_poses.front().orientation.w == 1.0 &&
                obj.mesh_poses.front().orientation.x == 0.0 &&
                obj.mesh_poses.front().orientation.y == 0.0 &&
                obj.mesh_poses.front().orientation.z == 0.0 &&
                obj.mesh_poses.front().position.x == m_pos.x &&
                obj.mesh_poses.front().position.y == m_pos.y &&
                obj.mesh_poses.front().position.z == m_pos.z;
   }

    // returns true if box was found in planning scene; false otherwise
    bool retrieveMesh(moveit_msgs::CollisionObject& obj)
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
        mesh_object.header.frame_id = "odom_combined";
        mesh_object.id = m_name;

        shape_msgs::Mesh mesh;

        // convert shapes::Mesh to shape_msgs::Mesh
        mesh.vertices.resize(m_mesh->vertex_count);
        for (int i = 0; i < m_mesh->vertex_count; ++i) {
            mesh.vertices[i].x = m_mesh->vertices[3 * i + 0];
            mesh.vertices[i].y = m_mesh->vertices[3 * i + 1];
            mesh.vertices[i].z = m_mesh->vertices[3 * i + 2];
        }

        mesh.triangles.resize(m_mesh->triangle_count);
        for (int i = 0; i < m_mesh->triangle_count; ++i) {
            mesh.triangles[i].vertex_indices[0] = m_mesh->triangles[3 * i + 0];
            mesh.triangles[i].vertex_indices[1] = m_mesh->triangles[3 * i + 1];
            mesh.triangles[i].vertex_indices[2] = m_mesh->triangles[3 * i + 2];
        }

        geometry_msgs::Pose mesh_pose;
        mesh_pose.position.x = m_pos.x;
        mesh_pose.position.y = m_pos.y;
        mesh_pose.position.z = m_pos.z;
        mesh_pose.orientation.w = 1.0;
        mesh_pose.orientation.x = 0.0;
        mesh_pose.orientation.y = 0.0;
        mesh_pose.orientation.z = 0.0;

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
        box_object.id = m_name;
        box_object.operation = moveit_msgs::CollisionObject::REMOVE;
        ROS_INFO("Removing mesh '%s'", m_name.c_str());
        m_collision_object_pub.publish(box_object);
        ros::Duration(1.0).sleep();
    }

    void moveBox(const moveit_msgs::CollisionObject& obj)
    {
        moveit_msgs::CollisionObject box = obj;
        box.header.seq = 0;
        box.header.stamp = ros::Time::now();
        box.primitive_poses.front().position.x = m_pos.x;
        box.primitive_poses.front().position.y = m_pos.y;
        box.primitive_poses.front().position.z = m_pos.z;
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
    if (argc < 6) {
        fprintf(stderr, "Usage: sync_mesh <x> <y> <z> <resource> <name>\n");
        return 1;
    }

    double pos_x = std::stod(argv[1]);
    double pos_y = std::stod(argv[2]);
    double pos_z = std::stod(argv[3]);
    std::string mesh_resource(argv[4]);
    std::string mesh_name(argv[5]);

    ros::init(argc, argv, "add_table");
    ros::NodeHandle nh;

    geometry_msgs::Point pos;
    pos.x = pos_x;
    pos.y = pos_y;
    pos.z = pos_z;

    MeshSynchronizer synchronizer;
    if (!synchronizer.init(mesh_name, mesh_resource, pos)) {
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
            // synchronize box
            synchronizer.sync();
            last_sync = now;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
