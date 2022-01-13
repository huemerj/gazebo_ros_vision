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
#include "utils.hpp"


namespace gazebo_ros
{
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using vision_msgs::msg::BoundingBox3D;
using ignition::math::AxisAlignedBox;

template<class T>
T Convert(const AxisAlignedBox &)
{
  T::ConversionNotImplemented;
}

template<>
BoundingBox3D Convert(const AxisAlignedBox & in)
{
  BoundingBox3D msg;
  msg.size = Convert<Vector3>(in.Size());
  return msg;
}

} // namespace gazebo_ros

namespace gazebo_ros_vision
{

using geometry_msgs::msg::Pose;
using vision_msgs::msg::Detection3DArray;
using vision_msgs::msg::BoundingBox3D;
using ignition::math::Pose3d;

class GazeboRosDetections : public gazebo::WorldPlugin
{
private:
  gazebo::physics::WorldPtr world_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detections_;

  std::set<std::string> model_whitelist_;

  /// Publishing rate in Hz
  double update_rate_;

  std::string frame_name_{"map"};

  gazebo::physics::ModelPtr reference_frame_;

  std::optional<double> max_distance_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Update event connection
  gazebo::event::ConnectionPtr update_connection_;

public:
  GazeboRosDetections() = default;
  virtual ~GazeboRosDetections() = default;

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
  {
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS & qos = ros_node_->get_qos();

    auto model_whitelist = sdf->FindElement("model_whitelist");
    if (model_whitelist) {
      model_whitelist_ = get_string_set(model_whitelist, "prefix");
    }
    if (!model_whitelist_.empty()) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "gazebo_ros_detections will publish poses of visible models with prefixes:");
      for (const auto & name : model_whitelist_) {
        RCLCPP_INFO(ros_node_->get_logger(), "* %s", name.c_str());
      }
    }

    auto update_rate = sdf->FindElement("update_rate");
    if (update_rate) {
      update_rate_ = update_rate->Get<double>();
    } else {
      RCLCPP_DEBUG(ros_node_->get_logger(), "plugin missing <update_rate>, defaults to 1Hz");
      update_rate_ = 1.0;
    }

    auto frame_name = sdf->FindElement("frame_name");
    if (frame_name) {
      frame_name_ = frame_name->Get<std::string>();
    } else {
      frame_name_ = "map";
    }
    if (frame_name_ != "map" && frame_name_ != "world") {
      reference_frame_ = world->ModelByName(frame_name_);
    }

    auto max_distance = sdf->FindElement("max_distance");
    if (max_distance) {
      max_distance_ = max_distance->Get<double>();
    }

    pub_detections_ = ros_node_->create_publisher<Detection3DArray>(
      "~/detections", qos.get_publisher_qos("~/detections", rclcpp::SensorDataQoS().reliable()));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosDetections::OnUpdate, this, std::placeholders::_1));
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    gazebo::common::Time current_time = info.simTime;

    if (current_time < last_update_time_) {
      RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
      last_update_time_ = current_time;
    }

    // rate control
    if (update_rate_ > 0 &&
      (current_time - last_update_time_).Double() < (1.0 / update_rate_))
    {
      return;
    }

    Detection3DArray msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

    for (const auto & model: world_->Models()) {
      auto name = model->GetName();
      if (!starts_with_one_of(name, model_whitelist_)) {
        continue;
      }

      auto pose = model->WorldPose();
      if (reference_frame_) {
        pose = reference_frame_->WorldPose().Inverse() * pose;
        if (max_distance_.has_value() && pose.Pos().Length() > max_distance_.value()) {
          continue;
        }
      }

      auto & detection = msg.detections.emplace_back();
      detection.id = name;
      detection.bbox = gazebo_ros::Convert<BoundingBox3D>(model->BoundingBox());
      auto & result = detection.results.emplace_back();
      result.hypothesis.score = 1.0;
      result.hypothesis.class_id = name;
      result.pose.pose = gazebo_ros::Convert<Pose>(pose);
    }

    pub_detections_->publish(msg);
    last_update_time_ = current_time;
  }
};

GZ_REGISTER_WORLD_PLUGIN(GazeboRosDetections)

} // namespace gazebo_ros_vision
