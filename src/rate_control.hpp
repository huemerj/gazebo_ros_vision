#pragma once
#include <set>
#include <string>
#include <sdf/Element.hh>

namespace gazebo_ros_vision
{

class RateControl
{
private:
  std::optional<double> rate_inv_;
  gazebo::common::Time last_t_;

public:
  RateControl() {}

  explicit RateControl(double rate)
  : rate_inv_(1.0 / rate)
  {

  }

  bool update(const gazebo::common::Time & t)
  {
    if (!rate_inv_.has_value()) {
      return true;
    }
    if (t < last_t_) {
      // jump back in time -> assume 0 dt
      last_t_ = t;
    }
    auto dt = (t - last_t_).Double();
    if (dt >= rate_inv_.value()) {
      last_t_ = t;
      return true;
    }
    return false;
  }
};

} // namespace gazebo_ros_vision
