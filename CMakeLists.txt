cmake_minimum_required(VERSION 3.5)
project(gazebo_ros_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(gazebo_ros_detections SHARED src/gazebo_ros_detections.cpp)
target_include_directories(gazebo_ros_detections PRIVATE src)
ament_target_dependencies(gazebo_ros_detections
  "gazebo_ros"
  "vision_msgs"
)

add_library(gazebo_ros_logical_camera SHARED src/gazebo_ros_logical_camera.cpp)
target_include_directories(gazebo_ros_logical_camera PRIVATE src)
ament_target_dependencies(gazebo_ros_logical_camera
  "gazebo_ros"
  "vision_msgs"
)

add_library(gazebo_ros_object_list SHARED src/gazebo_ros_object_list.cpp)
target_include_directories(gazebo_ros_object_list PRIVATE src)
ament_target_dependencies(gazebo_ros_object_list
  "gazebo_ros"
  "vision_msgs"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS gazebo_ros_detections gazebo_ros_logical_camera gazebo_ros_object_list
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()