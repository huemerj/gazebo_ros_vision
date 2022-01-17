#pragma once
#include <set>
#include <string>
#include <sdf/Element.hh>

namespace gazebo_ros_vision
{

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
    if (dt > rate_inv_.value()) {
      last_t_ = t;
      return true;
    }
    return false;
  }
};

} // namespace gazebo_ros_vision
