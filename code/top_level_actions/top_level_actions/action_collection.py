"""Collection of top level actions for the LLM Planner."""

from typing import cast

from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .llm_actions_node import LLMActions
from aidara_common.async_utils import AsyncServiceCall
from aidara_common.topic_utils import get_latest_msg_from_topic
from aidara_msgs.srv import (
    GeometricGrasp,
    TargetPose,
)


def say(message: str) -> None:
    """Relay a message to the user."""
    msg = String(data=message)
    LLMActions().user_feedback_pub.publish(msg)


def get_grasp_pose(object_description: str) -> PoseStamped | None:
    """Get the pose of an object on the table."""
    node = LLMActions()
    geometric_grasp_req = GeometricGrasp.Request()
    geometric_grasp_req.object_description.data = object_description
    try:
        grasp = cast(
            GeometricGrasp.Response,
            AsyncServiceCall.create(
                node,
                node.geometric_grasp_client,
                geometric_grasp_req,
            ).resolve_with_eh(),
        )
    except RuntimeError:
        return None

    return node.do_tf(
        source=grasp.pose,
        target_frame="table",
    )


def get_user_hand_pose() -> PoseStamped | None:
    """Return the hand pose of the users right hand in table frame."""
    node: LLMActions = LLMActions()
    try:
        hand_position = get_latest_msg_from_topic(
            node,
            "/hand_position",
            PointStamped,
        )
    except RuntimeError:
        return None

    hand_position = node.do_tf(
        source=hand_position,
        target_frame="table",
    )

    res = PoseStamped()
    res.pose.position = hand_position.point
    res.header.frame_id = hand_position.header.frame_id
    res.header.stamp = node.get_clock().now().to_msg()
    res.pose.orientation.x = 1.0
    res.pose.orientation.w = 0.0

    return res


def go_home() -> None:
    """Go to home position and neutral joint states."""
    node = LLMActions()
    move_gripper_to(node.home_pose)
    node.reset_joints()


def move_gripper_to(pose: PoseStamped | list[float]) -> None:
    """Move the gripper to a specific pose.

    In general this function is used to move the gripper to a specific pose with no
    particular frame in mind. However if a list is passed as pose, it is assumed to be
    a position in the table frame.
    """
    node = LLMActions()
    if isinstance(pose, list):
        pose = node.build_table_pose(pose)

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=pose),
    ).resolve_with_eh()


def move_gripper_by(dx: float, dy: float, dz: float) -> None:
    """Move the gripper by a specific amount."""
    node = LLMActions()

    target_pose = PoseStamped()
    target_pose.header.stamp = node.get_clock().now().to_msg()
    target_pose.header.frame_id = "eef"

    target_pose = node.do_tf(source=target_pose, target_frame="table")

    target_pose.pose.position.x += dx
    target_pose.pose.position.y += dy
    target_pose.pose.position.z += dz

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=target_pose),
    ).resolve_with_eh()


def open_gripper() -> None:
    """Open the gripper."""
    node = LLMActions()
    AsyncServiceCall.create(
        node,
        node.open_gripper_client,
        Trigger.Request(),
    ).resolve_with_eh(timeout=Duration(seconds=5))


def close_gripper() -> None:
    """Close the gripper."""
    node = LLMActions()
    AsyncServiceCall.create(
        node,
        node.close_gripper_client,
        Trigger.Request(),
    ).resolve_with_eh(timeout=Duration(seconds=5))


def relax() -> None:
    """Relax the robot's joints."""
    node = LLMActions()
    AsyncServiceCall.create(
        node,
        node.relax_client,
        Trigger.Request(),
    ).resolve_with_eh(timeout=Duration(seconds=5))


def tighten_up() -> None:
    """Tighten the robot's joints."""
    node = LLMActions()
    AsyncServiceCall.create(
        node,
        node.tighten_up_client,
        Trigger.Request(),
    ).resolve_with_eh(timeout=Duration(seconds=5))


def sleep(time_sec: float) -> None:
    """Sleep for the given duration."""
    node = LLMActions()
    duration = Duration(nanoseconds=int(time_sec * S_TO_NS))
    node.sleep(duration)


def shift_pose(pose: PoseStamped, dx: float, dy: float, dz: float) -> PoseStamped:
    """Return the pose shifted by [dx,dy,dz] in table frame."""
    node = LLMActions()
    pose = node.do_tf(source=pose, target_frame="table")

    pose.pose.position.x += float(dx)
    pose.pose.position.y += float(dy)
    pose.pose.position.z += float(dz)
    return pose
