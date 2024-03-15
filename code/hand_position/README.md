# hand_position

A ROS 2 Node to provide the position of the right palm.

The node listens to a zed camera, waits for a skeleton to be published,
and publishes the right palm position on '/hand_position'.

To start the node, run

```bash
ros2 run hand_position hand_position --zed-camera-name <zed_camera_name>
```
