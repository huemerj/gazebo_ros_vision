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
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

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

class GazeboRosDetections : public gazebo::ModelPlugin
{
private:
  gazebo::physics::ModelPtr model_;

  gazebo::physics::LinkPtr reference_link_;

  gazebo::sensors::SensorPtr reference_sensor_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detections_;

  RateControl rate_control_;

  std::string frame_name_{"map"};

  std::optional<double> max_distance_;

  std::unique_ptr<DetectionFilter> filter_;

  /// Update event connection
  gazebo::event::ConnectionPtr update_connection_;

public:
  GazeboRosDetections() = default;
  virtual ~GazeboRosDetections() = default;

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model_ = _model;
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    auto update_rate = _sdf->Get<double>("update_rate", 1.0);
    rate_control_ = RateControl(update_rate.first);
    if (!update_rate.second) {
      RCLCPP_DEBUG(ros_node_->get_logger(), "plugin missing <update_rate>, defaults to 1Hz");
    }

    auto frame_name_elem = _sdf->Get<std::string>("frame_name", model_->GetName());
    frame_name_ = frame_name_elem.first;
    if (frame_name_elem.second) {
      if (_sdf->Get<bool>("frame_is_sensor", false).first) {
        reference_sensor_ = gazebo::sensors::SensorManager::Instance()->GetSensor(frame_name_);
        if (!reference_sensor_) {
          RCLCPP_ERROR(ros_node_->get_logger(), "<frame_name> references invalid sensor: %s",frame_name_.c_str());
          return;
        }
      } else {
        reference_link_ = model_->GetLink(frame_name_);
        if (!reference_link_) {
          RCLCPP_ERROR(ros_node_->get_logger(), "<frame_name> references invalid link: %s ",frame_name_.c_str());
          return;
        }
      }
    }

    auto max_distance = _sdf->FindElement("max_distance");
    if (max_distance) {
      max_distance_ = max_distance->Get<double>();
    }

    filter_ = DetectionFilter::Load(_sdf);
    if (!filter_->model_whitelist_.empty()) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "%s will publish poses of visible models with prefixes:", handleName.c_str());
      for (const auto & name : filter_->model_whitelist_) {
        RCLCPP_INFO(ros_node_->get_logger(), "* %s", name.c_str());
      }
    }

    const gazebo_ros::QoS & qos = ros_node_->get_qos();
    pub_detections_ = ros_node_->create_publisher<Detection3DArray>(
      "~/detections", qos.get_publisher_qos("~/detections"));

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

    Pose3d reference_pose;
    if (reference_link_) {
      reference_pose = reference_link_->WorldPose();
    } else if (reference_sensor_) {
      auto parent_link = model_->GetLinkById(reference_sensor_->ParentId());
      reference_pose = parent_link->WorldPose() * reference_sensor_->Pose();
    }

    for (const auto & model: model_->GetWorld()->Models()) {
      ModelItem item;
      item.name = model->GetName();
      item.pose = reference_pose.Inverse() * model->WorldPose();
      if (max_distance_.has_value() && item.pose.Pos().Length() > max_distance_.value()) {
        continue;
      }

      auto detection = filter_->get_detection(item);
      if (detection.has_value()) {
        msg.detections.push_back(detection.value());
      }
    }
    pub_detections_->publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDetections)

} // namespace gazebo_ros_vision
