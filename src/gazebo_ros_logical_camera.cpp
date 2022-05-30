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
#include <random>
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
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;
using vision_msgs::msg::Detection3D;
using vision_msgs::msg::Detection3DArray;
using ignition::math::Pose3d;
using gazebo::sensors::LogicalCameraSensor;
using gazebo::sensors::LogicalCameraSensorPtr;

double frustum_volume(const LogicalCameraSensor & sensor)
{
  double hfov = sensor.HorizontalFOV().Radian();
  auto vfov = hfov / sensor.AspectRatio();
  auto width_scale = 2 * tan(hfov / 2);
  auto height_scale = 2 * tan(vfov / 2);

  auto width_far = width_scale * sensor.Far();
  auto height_far = height_scale * sensor.Far();

  auto width_near = width_scale * sensor.Near();
  auto height_near = height_scale * sensor.Near();

  return (width_far * height_far * sensor.Far() - width_near * height_near * sensor.Near()) / 3.0;
}

/// A plugin that publishes models seen by a logical camera as Detection3DArray.
class GazeboRosLogicalCamera : public gazebo::SensorPlugin
{
private:
  LogicalCameraSensorPtr camera_;

  std::string frame_name_;

  std::unique_ptr<DetectionFilter> filter_;

  // probability of false positive per cubic meter in view frustum
  double false_positive_p_;

  std::default_random_engine generator_;

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
    false_positive_p_ = _sdf->Get("false_positive_p", 0.0).first;

    auto qos = node->get_qos().get_publisher_qos("~/detections");
    pub_detection_array_ = node->create_publisher<Detection3DArray>("~/detections", qos);

    sensor_update_event_ =
      camera_->ConnectUpdated(std::bind(&GazeboRosLogicalCamera::OnUpdate, this));
  }

  size_t sample_no_of_false_positives()
  {
    auto vol = frustum_volume(*camera_);
    auto trials = static_cast<int>(std::ceil(vol));
    std::binomial_distribution<int> false_positives_distr(trials, false_positive_p_);
    return false_positives_distr(generator_);
  }

  Vector3d sample_frustum_position()
  {
    std::uniform_real_distribution<double> z_dist(0.0, 1.0);
    std::uniform_real_distribution<double> xy_dist(-1.0, 1.0);

    auto rz = z_dist(generator_);
    auto rx = xy_dist(generator_);
    auto ry = xy_dist(generator_);

    auto n = camera_->Near();
    auto f = camera_->Far();

    double hfov = camera_->HorizontalFOV().Radian();
    auto vfov = hfov / camera_->AspectRatio();

    // in optical frame
    auto z = n + (f - n) * rz;
    auto x = rx * tan(hfov / 2.0) * z;
    auto y = ry * tan(vfov / 2.0) * z;

    // but logical camera works in sensor frame
    return Vector3d(z, x, y);
  }

  Quaterniond sample_orientation()
  {
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
    auto roll = angle_dist(generator_);
    auto pitch = angle_dist(generator_);
    auto yaw = angle_dist(generator_);
    return Quaterniond(roll, pitch, yaw);
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

    // generate false positives
    auto n = sample_no_of_false_positives();
    for (size_t i = 0; i < n; i++) {
      Detection3D fp;
      fp.id = "fp" + std::to_string(i);
      fp.bbox.size = gazebo_ros::Convert<Vector3>(filter_->bounding_box_size());
      auto & result = fp.results.emplace_back();
      result.hypothesis.score = 1.0;
      result.hypothesis.class_id = fp.id;

      result.pose.pose.position = gazebo_ros::Convert<Point>(sample_frustum_position());
      result.pose.pose.orientation = gazebo_ros::Convert<Quaternion>(sample_orientation());
      filter_->pose_noise().SetCovariance(
        fp.results[0].pose,
        filter_->pose_noise_published_scale());
      msg.detections.push_back(fp);
    }

    pub_detection_array_->publish(msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLogicalCamera)

} // namespace gazebo_ros_vision
