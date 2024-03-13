# camera_calibration
This package includes the node "chessboard_calibration" that opens a ZED camera and calculates the homogeneous transform from camera to chessboard frame.
The transform then gets published to the topic /tf_static as a TransformStamped message.

# Run node
  ros2 run camera_calibration chessboard_calibration


# Call service
  ros2 service call /chessboard_calibration/calibrate_camera std_srvs/srv/Trigger
