"""Script to call the trajectory planner."""

import argparse
import json
import subprocess


def call_ros2_service(args: argparse.Namespace) -> None:
    """Call service with the given pose."""
    data = {
        "target": {
            "header": {"frame_id": args.frame},
            "pose": {
                "position": {"x": args.x, "y": args.y, "z": args.z},
                "orientation": {"x": args.qx, "y": args.qy, "z": args.qz, "w": args.qw},
            },
        },
    }

    json_data = json.dumps(data)
    cmd = f"ros2 service call /move_eef aidara_msgs/srv/TargetPose '{json_data}'"

    subprocess.run(cmd, shell=True, check=True)


def main() -> None:
    """Take pose from user and call the caller function."""
    parser = argparse.ArgumentParser(
        description="Service call to send robot to desired pose",
    )
    parser.add_argument("frame", type=str, help="frame")
    parser.add_argument("x", type=float, help="X coordinate")
    parser.add_argument("y", type=float, help="Y coordinate")
    parser.add_argument("z", type=float, help="Z coordinate")
    parser.add_argument("qx", type=float, help="X component of the quaternion")
    parser.add_argument("qy", type=float, help="Y component of the quaternion")
    parser.add_argument("qz", type=float, help="Z component of the quaternion")
    parser.add_argument("qw", type=float, help="W component of the quaternion")
    args = parser.parse_args()

    call_ros2_service(args)
