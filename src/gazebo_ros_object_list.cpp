// Copyright (C) 2022 AIT Austrian Institute of Technology GmbH

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

// http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <set>
#include <string>
#include <memory>

#include <vision_msgs/msg/detection3_d_array.hpp>
#include "detection_filter.hpp"
#include "rate_control.hpp"

namespace gazebo_ros_vision
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Vector3;
using vision_msgs::msg::Detection3DArray;
using ignition::math::Pose3d;
using ignition::math::Vector3d;

class GazeboRosObjectList : public gazebo::WorldPlugin
{
private:
  gazebo::physics::WorldPtr world_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detections_;

  RateControl rate_control_;

  std::string frame_name_{"map"};

  std::unique_ptr<DetectionFilter> filter_;

  gazebo::event::ConnectionPtr update_connection_;

public:
  GazeboRosObjectList() = default;
  virtual ~GazeboRosObjectList() = default;

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr _sdf)
  {
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    const gazebo_ros::QoS & qos = ros_node_->get_qos();

    auto update_rate = _sdf->Get<double>("update_rate", 1.0);
    rate_control_ = RateControl(update_rate.first);
    if (!update_rate.second) {
      RCLCPP_DEBUG(ros_node_->get_logger(), "plugin missing <update_rate>, defaults to 1Hz");
    }

    auto frame_name = _sdf->Get<std::string>("frame_name", "map").first;

    filter_ = DetectionFilter::Load(_sdf);
    if (!filter_->model_whitelist_.empty()) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "%s will publish poses of visible models with prefixes:", handleName.c_str());
      for (const auto & name : filter_->model_whitelist_) {
        RCLCPP_INFO(ros_node_->get_logger(), "* %s", name.c_str());
      }
    }

    pub_detections_ = ros_node_->create_publisher<Detection3DArray>(
      "~/detections", qos.get_publisher_qos("~/detections"));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosObjectList::OnUpdate, this, std::placeholders::_1));
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    if (!rate_control_.update(info.simTime)) {
      return;
    }

    Detection3DArray msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(info.simTime);
    for (const auto & model: world_->Models()) {
      ModelItem item;
      item.name = model->GetName();
      item.pose = model->WorldPose();

      auto detection = filter_->get_detection(item);
      if (detection.has_value()) {
        msg.detections.push_back(detection.value());
      }
    }
    pub_detections_->publish(msg);
  }
};

GZ_REGISTER_WORLD_PLUGIN(GazeboRosObjectList)

} // namespace gazebo_ros_vision
