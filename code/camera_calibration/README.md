# camera_calibration

This package includes the node "chessboard_calibration" that opens a ZED camera and calculates the homogeneous transform from camera to chessboard frame.
The transform then gets published to the topic /tf_static as a TransformStamped message.

# Run node

```bash
ros2 run camera_calibration chessboard_calibration
```

# Call service

The service needs to know how many inner corners there are on the chessboard in both dimensions (one less than the number of squares), and what the side length of the squares is.

```bash
ros2 service call /chessboard_calibration/calibrate_camera aidara_msgs/srv/CalibrateCamera "{camera_name: <str>, chessboard_width: <i>, chessboard_height: <j>, square_size: <k>}"
```
