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
#include <gazebo/physics/Link.hh>
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
#include "noise.hpp"

namespace gazebo_ros_vision
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Vector3;
using vision_msgs::msg::Detection3DArray;
using ignition::math::Pose3d;
using ignition::math::Vector3d;

class GazeboRosDetections : public gazebo::ModelPlugin
{
private:
  gazebo::physics::ModelPtr model_;

  gazebo::physics::LinkPtr reference_link_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detections_;

  RateControl rate_control_;

  std::string frame_name_{"map"};

  std::set<std::string> model_whitelist_;

  std::optional<double> max_distance_;

  PoseNoise pose_noise_;

  double pose_noise_published_scale_;

  Vector3d bounding_box_size_;

  Vector3d bounding_box_offset_;

  /// Update event connection
  gazebo::event::ConnectionPtr update_connection_;

public:
  GazeboRosDetections() = default;
  virtual ~GazeboRosDetections() = default;

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model_ = _model;
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    auto model_whitelist = _sdf->FindElement("model_whitelist");
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

    auto update_rate = _sdf->Get<double>("update_rate", 1.0);
    rate_control_ = RateControl(update_rate.first);
    if (!update_rate.second) {
      RCLCPP_DEBUG(ros_node_->get_logger(), "plugin missing <update_rate>, defaults to 1Hz");
    }

    auto frame_name_elem = _sdf->FindElement("frame_name");
    if (frame_name_elem) {
      frame_name_ = frame_name_elem->Get<std::string>();
      reference_link_ = model_->GetLink(frame_name_);
      if (!reference_link_) {
        RCLCPP_ERROR(ros_node_->get_logger(), "<frame_name> references invalid link");
        return;
      }
    } else {
      frame_name_ = model_->GetName();
    }
    frame_name_ = _sdf->Get<std::string>("frame_name", model_->GetName()).first;

    auto max_distance = _sdf->FindElement("max_distance");
    if (max_distance) {
      max_distance_ = max_distance->Get<double>();
    }

    auto pose_noise = _sdf->FindElement("pose_noise");
    if (pose_noise) {
      pose_noise_.Load(pose_noise);
    }
    pose_noise_published_scale_ = _sdf->Get("pose_noise_published_scale", 1.0).first;

    bounding_box_size_ = _sdf->Get<Vector3d>("bounding_box_size", Vector3d::One).first;
    bounding_box_offset_ = _sdf->Get<Vector3d>("bounding_box_offset", Vector3d::Zero).first;

    const gazebo_ros::QoS & qos = ros_node_->get_qos();
    pub_detections_ = ros_node_->create_publisher<Detection3DArray>(
      "~/detections", qos.get_publisher_qos("~/detections", rclcpp::SensorDataQoS().reliable()));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosDetections::OnUpdate, this, std::placeholders::_1));
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    if (!rate_control_.update(info.simTime)) {
      return;
    }

    Detection3DArray msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(info.simTime);

    for (const auto & model: model_->GetWorld()->Models()) {
      auto name = model->GetName();
      if (!starts_with_one_of(name, model_whitelist_)) {
        continue;
      }

      auto pose = model->WorldPose();

      Pose3d reference_pose;
      if (reference_link_) {
        reference_pose = reference_link_->WorldPose();
      } else {
        reference_pose = model_->WorldPose();
      }
      pose = reference_pose.Inverse() * pose;
      if (max_distance_.has_value() && pose.Pos().Length() > max_distance_.value()) {
        continue;
      }

      auto & det = msg.detections.emplace_back();
      det.id = name;
      det.bbox.size = gazebo_ros::Convert<Vector3>(bounding_box_size_);
      det.bbox.center.position.x = bounding_box_offset_.X();
      det.bbox.center.position.y = bounding_box_offset_.Y();
      det.bbox.center.position.z = bounding_box_offset_.Z();
      auto & result = det.results.emplace_back();
      result.hypothesis.score = 1.0;
      result.hypothesis.class_id = name;

      pose = pose_noise_.Apply(pose);
      result.pose.pose = gazebo_ros::Convert<Pose>(pose);
      pose_noise_.SetCovariance(det.results[0].pose, pose_noise_published_scale_);
    }

    pub_detections_->publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDetections)

} // namespace gazebo_ros_vision
