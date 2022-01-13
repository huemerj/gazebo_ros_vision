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
#include <sdf/Element.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <string>
#include <set>
#include "noise.hpp"
#include "utils.hpp"

namespace gazebo_ros_vision
{

using vision_msgs::msg::Detection3DArray;
using ignition::math::Pose3d;

/// A plugin that publishes models seen by a logical camera as Detection3DArray.
class GazeboRosLogicalCamera : public gazebo::SensorPlugin
{
private:
  gazebo::sensors::LogicalCameraSensorPtr camera_;

  std::string frame_name_;

  std::set<std::string> model_whitelist_;

  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detection_array_;

  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Scale simulated pose noise covariance by this value to obtain published covariance
  double pose_noise_published_scale_;

  /// Noise to apply to detection poses
  PoseNoise pose_noise_;

public:
  GazeboRosLogicalCamera() = default;

  virtual ~GazeboRosLogicalCamera() = default;

protected:
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
  {
    auto node = gazebo_ros::Node::Get(_sdf);
    camera_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
    if (!camera_) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Plugin must be attached to sensor of type `logical_camera`");
      return;
    }

    frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

    auto model_whitelist = _sdf->FindElement("model_whitelist");
    if (model_whitelist) {
      model_whitelist_ = get_string_set(model_whitelist, "prefix");
    }
    if (!model_whitelist_.empty()) {
      RCLCPP_INFO(
        node->get_logger(),
        "gazebo_ros_logical_camera will publish poses of visible models with prefixes:");
      for (const auto & name : model_whitelist_) {
        RCLCPP_INFO(node->get_logger(), "* %s", name.c_str());
      }
    }

    auto pose_noise = _sdf->FindElement("pose_noise");
    if (pose_noise) {
      pose_noise_.Load(pose_noise);
    }

    auto pose_noise_published_scale = _sdf->FindElement("pose_noise_published_scale");
    if (pose_noise_published_scale) {
      pose_noise_published_scale_ = pose_noise_published_scale->Get<double>();
    } else {
      pose_noise_published_scale_ = 1.0;
    }

    auto qos = node->get_qos().get_publisher_qos(
      "~/detections",
      rclcpp::SensorDataQoS().reliable());
    pub_detection_array_ = node->create_publisher<Detection3DArray>("~/detections", qos);

    sensor_update_event_ =
      camera_->ConnectUpdated(std::bind(&GazeboRosLogicalCamera::OnUpdate, this));
  }

  void OnUpdate()
  {
    if (!camera_->IsActive()) {
      return;
    }
    Detection3DArray msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
      camera_->LastUpdateTime());
    auto image = camera_->Image();
    for (const auto & model : image.model()) {
      if (!starts_with_one_of(model.name(), model_whitelist_)) {
        continue;
      }
      auto & det = msg.detections.emplace_back();
      det.id = model.name();
      det.results.resize(1);

      Pose3d pose = gazebo::msgs::ConvertIgn(model.pose());
      pose = pose_noise_.Apply(pose);
      det.results[0].hypothesis.score = 1.0;
      det.bbox.size.x = 1.0;
      det.bbox.size.y = 1.0;
      det.bbox.size.z = 1.0;
      det.results[0].pose.pose =
        gazebo_ros::Convert<geometry_msgs::msg::Pose>(
        pose);
      pose_noise_.SetCovariance(det.results[0].pose, pose_noise_published_scale_);
    }
    pub_detection_array_->publish(msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLogicalCamera)

} // namespace gazebo_ros_vision
