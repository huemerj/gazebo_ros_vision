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
#pragma once

#include <set>
#include <string>
#include <memory>
#include <random>
#include <ignition/math/Pose3.hh>
#include <vision_msgs/msg/detection3_d.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <sdf/sdf.hh>
#include "noise.hpp"

namespace gazebo_ros_vision
{
using ignition::math::Pose3d;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::PoseWithCovariance;
using vision_msgs::msg::Detection3D;

struct ModelItem
{
  std::string name;
  ignition::math::Pose3d pose;
};

std::set<std::string> get_string_set(sdf::ElementPtr elem, const std::string & sub_elem_name)
{
  std::set<std::string> result;
  auto i = elem->GetFirstElement();
  while (i && i->GetName() == sub_elem_name) {
    result.insert(i->Get<std::string>());
    i = i->GetNextElement(sub_elem_name);
  }
  return result;
}

bool starts_with_one_of(const std::string & name, const std::set<std::string> & prefixes)
{
  if (prefixes.empty()) {
    return true;
  }
  return std::any_of(
    prefixes.cbegin(), prefixes.cend(), [&name](const auto & prefix) {
      return name.rfind(prefix, 0) == 0;
    });
}

class DetectionFilter
{
private:
  PoseNoise pose_noise_;

  double pose_noise_published_scale_;

  Vector3d bounding_box_size_;

  Vector3d bounding_box_offset_;

  double recall_;

  std::random_device rd_;

  std::mt19937 gen_;

  std::uniform_real_distribution<> dist_;

public:
  std::set<std::string> model_whitelist_;

  DetectionFilter() : recall_(1.0), gen_(rd_()), dist_(0, 1) {
    
  }

  static std::unique_ptr<DetectionFilter> Load(sdf::ElementPtr _sdf)
  {
    auto filter = std::make_unique<DetectionFilter>();
    auto model_whitelist = _sdf->FindElement("model_whitelist");
    if (model_whitelist) {
      filter->model_whitelist_ = get_string_set(model_whitelist, "prefix");
    }

    auto pose_noise = _sdf->FindElement("pose_noise");
    if (pose_noise) {
      filter->pose_noise_.Load(pose_noise);
    }
    filter->pose_noise_published_scale_ = _sdf->Get("pose_noise_published_scale", 1.0).first;
    filter->bounding_box_size_ = _sdf->Get<Vector3d>("bounding_box_size", Vector3d::One).first;
    filter->bounding_box_offset_ = _sdf->Get<Vector3d>("bounding_box_offset", Vector3d::Zero).first;
    filter->recall_ = _sdf->Get("recall", 1.0).first;
    return filter;
  }

  std::optional<Detection3D> get_detection(const ModelItem & model)
  {
    if (!starts_with_one_of(model.name, model_whitelist_)) {
      return std::nullopt;
    }
    if (dist_(gen_) > recall_) {
      return std::nullopt;
    }

    Detection3D det;
    det.id = model.name;
    det.bbox.size = gazebo_ros::Convert<Vector3>(bounding_box_size_);
    det.bbox.center.position.x = bounding_box_offset_.X();
    det.bbox.center.position.y = bounding_box_offset_.Y();
    det.bbox.center.position.z = bounding_box_offset_.Z();
    auto & result = det.results.emplace_back();
    result.hypothesis.score = 1.0;
    result.hypothesis.class_id = model.name;

    auto pose = pose_noise_.Apply(model.pose);
    result.pose.pose = gazebo_ros::Convert<Pose>(pose);
    pose_noise_.SetCovariance(det.results[0].pose, pose_noise_published_scale_);
    return det;
  }
};

} // namespace gazebo_ros_vision
