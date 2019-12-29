/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <map>
#include <set>
#include <string>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include <ignition/math/Vector3.hh>

#include <ros/ros.h>

#include <sdf/sdf.hh>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace airi
{
namespace plugins
{

class BumperPlugin : public gazebo::SensorPlugin
{
public:
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf)
  {
    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    sensor_ = dynamic_pointer_cast<gazebo::sensors::ContactSensor>(parent);
    if (!sensor_) {
      ROS_FATAL_NAMED(sensor_->Name(), "Plugin not attached to a contact sensor");
      return;
    }

    gazebo::physics::WorldPtr world = gazebo::physics::get_world(sensor_->WorldName());
    link_ = dynamic_pointer_cast<gazebo::physics::Link>(world->EntityByName(sensor_->ParentName()));
    if (!link_) {
      ROS_FATAL_STREAM_NAMED(sensor_->Name(), "Parent '" << sensor_->ParentName() << "' is not a link.");
      return;
    }

    if (sdf->HasElement("displacement_axis")) {
      displacement_axis_ = sdf->GetElement("displacement_axis")->Get<ignition::math::Vector3d>().Normalize();
    } else {
      ROS_WARN_NAMED(sensor_->Name(), "No displacement axis was specified, using default.");
    }
    ROS_INFO_STREAM_NAMED(sensor_->Name(), "Using [" << displacement_axis_ << "] as displacement axis");

    if (sdf->HasElement("contact_offset")) {
      contact_offset_ = sdf->GetElement("contact_offset")->Get<double>();
    } else {
      ROS_WARN_NAMED(sensor_->Name(), "No contact offset was specified, using default.");
    }
    ROS_INFO_STREAM_NAMED(sensor_->Name(), "Using " << contact_offset_ << " as contact offset");

    if (sdf->HasElement("sample_grid")) {
      sample_grid_ = sdf->GetElement("sample_grid")->Get<ignition::math::Vector3d>();
      ROS_INFO_STREAM_NAMED(sensor_->Name(), "Using [" << sample_grid_ << "] as sample grid.");
    } else {
      ROS_WARN_NAMED(sensor_->Name(), "No sample grid was specified, using all vertices.");
    }

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM_NAMED(sensor_->Name(), "A ROS node for Gazebo has not been initialized, "
                             << "unable to load plugin. Load the Gazebo system plugin "
                             << "('libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    nh_.reset(new ros::NodeHandle());

    if (sdf->HasElement("topic")) {
      topic_name_ = sdf->GetElement("topic")->Get<std::string>();
    } else {
      ROS_WARN_NAMED(sensor_->Name(), "No topic name was specified, using default.");
    }

    ROS_INFO_STREAM_NAMED(sensor_->Name(), "Publishing collisions to " << topic_name_);
    pub_ = nh_->advertise<sensor_msgs::PointCloud2>(topic_name_, 1);

    world_update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&BumperPlugin::OnWorldUpdate, this));
  }

  void OnWorldUpdate() {
    auto * manager = gazebo::common::MeshManager::Instance();
    const std::string collision_namespace = link_->GetScopedName() + "::";
    for (auto i = 0; i < sensor_->GetCollisionCount(); ++i) {
      std::string collision_name = sensor_->GetCollisionName(i);
      if (collision_name.substr(0, collision_namespace.size()) == collision_namespace) {
        collision_name.erase(0, collision_namespace.size());
      }
      gazebo::physics::CollisionPtr collision = link_->GetCollision(collision_name);
      if (!collision) {
        ROS_FATAL_STREAM_NAMED(sensor_->Name(), "'" << collision_name
                               << "' collision geometry not found.");
        world_update_connection_.reset();
        return;
      }
      auto shape = collision->GetShape();
      if (!shape->HasType(gazebo::physics::Shape::MESH_SHAPE)) {
        ROS_FATAL_STREAM_NAMED(sensor_->Name(), "'" << collision->GetName()
                               << "' collision geometry is not a mesh.");
        world_update_connection_.reset();
        return;
      }
      GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
      auto mesh_shape = dynamic_pointer_cast<gazebo::physics::MeshShape>(shape);

      const gazebo::common::Mesh * mesh = manager->GetMesh(mesh_shape->GetMeshURI());
      assert(mesh != nullptr);

      unsigned int num_points = 0;
      std::map<unsigned int, std::vector<unsigned int>> indices_per_submesh;
      for (auto i = 0; i < mesh->GetSubMeshCount(); ++i) {
        const gazebo::common::SubMesh * submesh = mesh->GetSubMesh(i);
        assert(submesh != nullptr);
        for (auto j = 0; j < submesh->GetIndexCount(); ++j) {
          unsigned int index = submesh->GetIndex(j);
          if (submesh->Normal(index).Dot(displacement_axis_) > 0) {
            indices_per_submesh[i].push_back(index);
            num_points++;
          }
        }
      }

      sensor_msgs::PointCloud2 & cloud =
        sampled_collisions_[collision->GetScopedName()];
      cloud.height = 1;


      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(3,
                                    "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32);
      modifier.resize(num_points);

      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
      const ignition::math::Vector3d scale = mesh_shape->Size();
      const ignition::math::Pose3d link_to_collision_transform = collision->RelativePose();
      for (const auto & kv : indices_per_submesh) {
        const gazebo::common::SubMesh * submesh = mesh->GetSubMesh(kv.first);
        for (const auto index : kv.second) {
          const ignition::math::Vector3d vertex =
            link_to_collision_transform.CoordPositionAdd(scale * submesh->Vertex(index) +
                                                         displacement_axis_ * contact_offset_);
          *iter_x = vertex.X(); *iter_y = vertex.Y(); *iter_z = vertex.Z();
          ++iter_x; ++iter_y; ++iter_z;
        }
      }

      if (sample_grid_ != ignition::math::Vector3d::Zero) {
        pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
        pcl_conversions::moveToPCL(cloud, *pcl_cloud);
        pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
        grid.setInputCloud(pcl_cloud);
        grid.setLeafSize(sample_grid_.X(), sample_grid_.Y(), sample_grid_.Z());
        pcl::PCLPointCloud2 pcl_cloud_filtered;
        grid.filter(pcl_cloud_filtered);
        pcl_conversions::moveFromPCL(pcl_cloud_filtered, cloud);
      }
    }

    sensor_update_connection_ =
      sensor_->ConnectUpdated(std::bind(&BumperPlugin::OnSensorUpdate, this));
    sensor_->SetActive(true);

    world_update_connection_.reset();
  }

  void OnSensorUpdate() {
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(3,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32);
    cloud.height = cloud.width = 0;

    std::set<std::string> active_collisions;
    const gazebo::msgs::Contacts contacts = sensor_->Contacts();
    for (auto i = 0; i < contacts.contact_size(); ++i) {
      const gazebo::msgs::Contact & contact = contacts.contact(i);
      auto it = sampled_collisions_.find(contact.collision1());
      if (it == sampled_collisions_.end()) {
        it = sampled_collisions_.find(contact.collision2());
        assert(it != sampled_collisions_.end());
      }
      const std::string & name = it->first;
      if (active_collisions.count(name) == 0U) {
        const sensor_msgs::PointCloud2 & sampled_collision = it->second;
        assert(pcl::concatenatePointCloud(cloud, sampled_collision, cloud));
        active_collisions.insert(name);
      }
    }
    cloud.header.frame_id = link_->GetName();
    cloud.header.stamp.sec = contacts.time().sec();
    cloud.header.stamp.nsec = contacts.time().nsec();
    pub_.publish(cloud);
  }

private:
  ros::NodeHandlePtr nh_;
  ros::Publisher pub_;

  gazebo::physics::LinkPtr link_;
  gazebo::sensors::ContactSensorPtr sensor_;

  std::string topic_name_{"bumper/collisions"};
  double contact_offset_{0.};
  ignition::math::Vector3d displacement_axis_{ignition::math::Vector3d::UnitX};
  ignition::math::Vector3d sample_grid_{ignition::math::Vector3d::Zero};
  std::map<std::string, sensor_msgs::PointCloud2> sampled_collisions_;

  gazebo::event::ConnectionPtr sensor_update_connection_;
  gazebo::event::ConnectionPtr world_update_connection_;
};

GZ_REGISTER_SENSOR_PLUGIN(BumperPlugin)

}  // namespace plugins

}  // namespace airi
