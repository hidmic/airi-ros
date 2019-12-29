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

    if (sdf->HasElement("axis")) {
      displacement_axis_ = sdf->GetElement("axis")->Get<ignition::math::Vector3d>();
    } else {
      ROS_WARN_NAMED(sensor_->Name(), "No displacement axis was specified, using default.");
    }
    ROS_INFO_STREAM_NAMED(sensor_->Name(), "Using [" << displacement_axis_ << "] as displacement axis");

    auto * manager = ignition::common::MeshManager::Instance();
    for (const auto & collision : link_->GetCollisions()) {
      if(collision->GetShapeType() != gazebo::physics::Shape::MESH_SHAPE) {
        ROS_FATAL_NAMED(sensor_->Name(), "Rigid body collision is not a mesh.");
        return;
      }

      sensor_msgs::PointCloud2 & cloud = sampled_collisions_[collision->GetName()];
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(3,
                                    "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32);
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

      auto shape = dynamic_pointer_cast<gazebo::physics::MeshShape>(collision->GetShape());
      const ignition::common::Mesh * mesh  = manager->MeshByName(shape->GetMeshURI());
      for (auto i = 0; i < mesh->SubMeshCount(); ++i) {
        std::weak_ptr<ignition::common::SubMesh> ref = mesh->SubMeshByIndex(i);
        if (!ref.expired()) {
          std::shared_ptr<ignition::common::SubMesh> submesh = ref.lock();
          for (auto j = 0; j < submesh->IndexCount(); ++j) {
            if (submesh->Normal(submesh->Index(j)).Dot(displacement_axis_) > 0) {
              const ignition::math::Vector3d vertex = submesh->Vertex(submesh->Index(j));
              *iter_x = vertex.X(); *iter_y = vertex.Y(); *iter_z = vertex.Z();
              ++iter_x; ++iter_y; ++iter_z; ++cloud.width;
            }
          }
        }
      }
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

    update_connection_ = sensor_->ConnectUpdated(std::bind(&BumperPlugin::OnUpdate, this));

    sensor_->SetActive(true);
  }

  void OnUpdate() {
    const gazebo::msgs::Contacts contacts = sensor_->Contacts();

    sensor_msgs::PointCloud2 cloud;
    cloud.height = cloud.width = 0;
    std::set<std::string> active_collisions;
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
  ignition::math::Vector3d displacement_axis_{ignition::math::Vector3d::UnitX};
  std::map<std::string, sensor_msgs::PointCloud2> sampled_collisions_;

  gazebo::event::ConnectionPtr update_connection_;
};

GZ_REGISTER_SENSOR_PLUGIN(BumperPlugin)

}  // namespace plugins

}  // namespace airi
