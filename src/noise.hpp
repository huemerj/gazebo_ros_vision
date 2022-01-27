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
#include <gazebo/sensors/Noise.hh>
#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <memory>
#include <string>

namespace gazebo_ros_vision
{
using gazebo::sensors::NoisePtr;
using gazebo::sensors::NoiseFactory;
using gazebo::sensors::Noise;
using gazebo::sensors::GaussianNoiseModel;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TwistWithCovariance;
using ignition::math::Quaterniond;
using ignition::math::Pose3d;
using ignition::math::Vector3d;

inline double sigma(const NoisePtr & noise)
{
  auto gaussian_noise = dynamic_cast<GaussianNoiseModel *>(noise.get());
  if (gaussian_noise) {
    return gaussian_noise->GetStdDev();
  }
  return 0.0;
}

sdf::ElementPtr GetNested(sdf::ElementPtr sdf, std::string name0, std::string name1) {
    if (!sdf->HasElement(name0)) {
      return nullptr;
    }
    auto first = sdf->GetElement(name0);
    if (!first->HasElement(name1)) {
      return nullptr;
    }
    return first->GetElement(name1);
}

class VectorNoise
{
private:
  NoisePtr x_;
  NoisePtr y_;
  NoisePtr z_;

public:
  VectorNoise()
  : x_(new Noise(Noise::NONE)), y_(new Noise(Noise::NONE)), z_(new Noise(Noise::NONE)) {}

  void Load(sdf::ElementPtr sdf)
  {
    auto x_elem = GetNested(sdf, "x", "noise");
    if (x_elem) {
      x_ = NoiseFactory::NewNoiseModel(x_elem);
    }
    auto y_elem = GetNested(sdf, "y", "noise");
    if (y_elem) {
      y_ = NoiseFactory::NewNoiseModel(y_elem);
    }
    auto z_elem = GetNested(sdf, "z", "noise");
    if (z_elem) {
      z_ = NoiseFactory::NewNoiseModel(z_elem);
    }
  }

  double sigma_x() const
  {
    return sigma(x_);
  }

  double sigma_y() const
  {
    return sigma(y_);
  }

  double sigma_z() const
  {
    return sigma(z_);
  }

  Vector3d Apply(Vector3d & v)
  {
    auto x = x_->Apply(v.X());
    auto y = y_->Apply(v.Y());
    auto z = z_->Apply(v.Z());
    return Vector3d(x, y, z);
  }

  Quaterniond Apply(const Quaterniond & q)
  {
    auto v = q.Euler();
    auto r = x_->Apply(v.X());
    auto p = y_->Apply(v.Y());
    auto y = z_->Apply(v.Z());
    return ignition::math::Quaterniond::EulerToQuaternion(r, p, y);
  }
};

class PoseNoise
{
private:
  VectorNoise pos_noise_;
  VectorNoise rot_noise_;

public:
  PoseNoise()
  : pos_noise_(), rot_noise_() {}

  void Load(sdf::ElementPtr sdf)
  {
    if (sdf->HasElement("position")) {
      pos_noise_.Load(sdf->GetElement("position"));
    }
    if (sdf->HasElement("orientation")) {
      rot_noise_.Load(sdf->GetElement("orientation"));
    }
  }

  Pose3d Apply(Pose3d pose)
  {
    auto rot = rot_noise_.Apply(pose.Rot());
    rot.Normalize();
    return Pose3d(pos_noise_.Apply(pose.Pos()), rot);
  }

  void SetCovariance(PoseWithCovariance & pose, double scale = 1.0) const
  {
    auto sx = pos_noise_.sigma_x() * scale;
    auto sy = pos_noise_.sigma_y() * scale;
    auto sz = pos_noise_.sigma_z() * scale;
    auto sroll = rot_noise_.sigma_x() * scale;
    auto spitch = rot_noise_.sigma_y() * scale;
    auto syaw = rot_noise_.sigma_z() * scale;
    pose.covariance[0] = sx * sx;
    pose.covariance[7] = sy * sy;
    pose.covariance[14] = sz * sz;
    pose.covariance[21] = sroll * sroll;
    pose.covariance[28] = spitch * spitch;
    pose.covariance[35] = syaw * syaw;
  }
};

class TwistNoise
{
private:
  VectorNoise vel_noise_;
  VectorNoise rot_noise_;

public:
  TwistNoise()
  : vel_noise_(), rot_noise_() {}

  void Load(sdf::ElementPtr sdf)
  {
    if (sdf->HasElement("velocity")) {
      vel_noise_.Load(sdf->GetElement("velocity"));
    }
    if (sdf->HasElement("rotation")) {
      rot_noise_.Load(sdf->GetElement("rotation"));
    }
  }

  Vector3d ApplyVel(Vector3d v)
  {
    return vel_noise_.Apply(v);
  }

  Vector3d ApplyRot(Vector3d v)
  {
    return rot_noise_.Apply(v);
  }

  void GetCovariance(TwistWithCovariance & twist) const
  {
    auto sx = vel_noise_.sigma_x();
    auto sy = vel_noise_.sigma_y();
    auto sz = vel_noise_.sigma_z();
    auto srx = rot_noise_.sigma_x();
    auto sry = rot_noise_.sigma_y();
    auto srz = rot_noise_.sigma_z();
    twist.covariance[0] = sx * sx;
    twist.covariance[7] = sy * sy;
    twist.covariance[14] = sz * sz;
    twist.covariance[21] = srx * srx;
    twist.covariance[28] = sry * sry;
    twist.covariance[35] = srz * srz;
  }
};

} // namespace gazebo_ros_vision
