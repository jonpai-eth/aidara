# camera_calibration
This package includes the node "calibration_node" that opens a ZED camera and calculates the homogeneous transform from camera to chessboard frame.
The transform then gets published to the topic /tf_static as a TransformStamped message.

# Run node
  ros2 run camera_calibration calibration_node


# Call service
  ros2 service call /calibrate_camera std_srvs/srv/Trigger
