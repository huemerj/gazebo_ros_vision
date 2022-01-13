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

} // namespace gazebo_ros_vision
