Publish simulated Gazebo model poses as `vision_msgs/msg/Detection3DArray`.

* `gazebo_ros_logical_camera` publishes model poses seen by a logical camera (SensorPlugin)
* `gazebo_ros_detections` publishes model poses relative to a model (ModelPlugin)
* `gazebo_ros_object_list` publishes model poses in a world (WorldPlugin)

For usage examples check `test/example.world`, or run the world in Gazebo:

```bash
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so test/example.world
```
