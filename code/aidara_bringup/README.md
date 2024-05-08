# aidara_bringup
This package provides all launch files to launch the aidara project. So far, this package doesn't launch the controller for the robot.
The aidara project is setup to run on three different PCs. One PC exclusively runs the controller. The other two PCs are used to distribute the workload. 

**Launch trajectory planner and speech_to_text node**

Depending on your robot, use franka_launch.py or staubli_launch.py as launch file name.

```bash
ros2 launch aidara_bringup <launch file name>
```
**Launch geometric_grasp, tf2_service, text_to_speech, hand_position, llm_planning and vision_launch and calibrate the cameras**

A camera is only launched if there is a serial number provided

```bash
ros2 launch aidara_bringup aidara_launch.py llm:=<llm planner> is_dry_run:=<is dry run> examples_vision_mode:=<vision mode> request_vision_mode:=<vision mode> robot:=<robot> hand_tracking_camera_name:=<camera name> zed_top_sn:=<serial number> zed_right_sn:=<serial number> zed_left_sn:=<serial number>
```
