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

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <string>
#include <memory>
#include "detection_filter.hpp"

namespace gazebo_ros_vision
{
using geometry_msgs::msg::Vector3;
using vision_msgs::msg::Detection3DArray;
using ignition::math::Pose3d;

/// A plugin that publishes models seen by a logical camera as Detection3DArray.
class GazeboRosLogicalCamera : public gazebo::SensorPlugin
{
private:
  gazebo::sensors::LogicalCameraSensorPtr camera_;

  std::string frame_name_;

  std::unique_ptr<DetectionFilter> filter_;

  rclcpp::Publisher<Detection3DArray>::SharedPtr pub_detection_array_;

  gazebo::event::ConnectionPtr sensor_update_event_;

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

    filter_ = DetectionFilter::Load(_sdf);
    if (!filter_->model_whitelist_.empty()) {
      RCLCPP_INFO(
        node->get_logger(),
        "%s will publish poses of visible models with prefixes:", handleName.c_str());
      for (const auto & name : filter_->model_whitelist_) {
        RCLCPP_INFO(node->get_logger(), "* %s", name.c_str());
      }
    }

    auto qos = node->get_qos().get_publisher_qos("~/detections");
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
      ModelItem item;
      item.name = model.name();
      item.pose = gazebo::msgs::ConvertIgn(model.pose());
      auto detection = filter_->get_detection(item);
      if (detection.has_value()) {
        msg.detections.push_back(detection.value());
      }
    }
    pub_detection_array_->publish(msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLogicalCamera)

} // namespace gazebo_ros_vision
