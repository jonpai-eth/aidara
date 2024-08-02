# aidara_bringup

This package provides all launch files to launch the aidara project Alfred is setup to run on at least two PCs such that one PC exclusively runs the controller.

### Computer 1

- trajectory planner
- speech_to_text
- text_to_speech

```bash
ros2 launch aidara_bringup <franka_launch.py or staubli_launch.py>
```

### Computer 2+

- llm_planner
- geometric_grasp
- hand_position

```bash
ros2 launch aidara_bringup aidara_launch.py llm:=<llm_planner> prompt_version:=<prompt_version> is_dry_run:=<is_dry_run> examples_vision_mode:=<vision_mode> request_vision_mode:=<vision_mode> robot:=<robot> hand_tracking_camera_name:=<camera_name>
```

- cameras
- camera calibration

```bash
ros2 launch aidara_bringup vision_launch.py zed_top_sn:=<serial_number> zed_right_sn:=<serial_number> zed_left_sn:=<serial_number> chessboard_width:=<chessboard_width> chessboard_height:=<chessboard_height> square_size:=<square_size>

```
