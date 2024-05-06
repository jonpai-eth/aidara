"""Collection of top level actions for the LLM Planner."""

import copy
from typing import cast

from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .llm_actions_node import LLMActions
from aidara_common.async_utils import AsyncServiceCall
from aidara_common.tf_utils import convert
from aidara_msgs.srv import GeometricGrasp, TargetPose


def say(message: str) -> None:
    """Relay a message to the user."""
    msg = String(data=message)
    LLMActions().user_feedback_pub.publish(msg)


def pick_object_from_table(object_description: str) -> None:
    """Pick an object from the table."""
    node = LLMActions()

    open_gripper_call = AsyncServiceCall.create(
        node,
        node.open_gripper_client,
        Trigger.Request(),
    )

    geometric_grasp_req = GeometricGrasp.Request()
    geometric_grasp_req.object_description.data = object_description
    grasp_pose = cast(
        GeometricGrasp.Response,
        AsyncServiceCall.create(
            node,
            node.geometric_grasp_client,
            geometric_grasp_req,
        ).resolve_with_eh(),
    ).pose

    grasp_frame = cast(TransformStamped, convert(grasp_pose, "grasp"))
    node.publish_static_transform(grasp_frame)

    pre_grasp_pose = PoseStamped()
    pre_grasp_pose.header.frame_id = "grasp"
    pre_grasp_pose.pose.position.z = -0.1

    open_gripper_call.resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=pre_grasp_pose),
    ).resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=grasp_pose),
    ).resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.close_gripper_client,
        Trigger.Request(),
    ).resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=pre_grasp_pose),
    ).resolve_with_eh()


def place_object_on_table_at(x: float, y: float) -> None:
    """Place on object on the table."""
    node = LLMActions()

    place_pose = PoseStamped()
    place_pose.header.frame_id = "table"
    place_pose.header.stamp = node.get_clock().now().to_msg()
    place_pose.pose.position.x = x
    place_pose.pose.position.y = y

    place_frame = cast(TransformStamped, convert(place_pose, "grasp"))
    node.publish_static_transform(place_frame)

    pre_place_pose = copy.deepcopy(place_pose)
    pre_place_pose.pose.position.z += 0.1

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=pre_place_pose),
    ).resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=place_pose),
    ).resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.open_gripper_client,
        Trigger.Request(),
    ).resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=pre_place_pose),
    ).resolve_with_eh()


def take_from_hand() -> None:
    """Take an object from the user's hand."""
    node = LLMActions()

    handover_pose = node.get_handover_pose()

    open_gripper_call = AsyncServiceCall.create(
        node,
        node.open_gripper_client,
        Trigger.Request(),
    )

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=handover_pose),
    ).resolve_with_eh()

    open_gripper_call.resolve_with_eh()

    node.sleep(Duration(seconds=4))

    AsyncServiceCall.create(
        node,
        node.close_gripper_client,
        Trigger.Request(),
    ).resolve_with_eh()


def give_to_hand() -> None:
    """Give an object to the user's hand."""
    node = LLMActions()

    handover_pose = node.get_handover_pose()

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=handover_pose),
    ).resolve_with_eh()

    AsyncServiceCall.create(
        node,
        node.open_gripper_client,
        Trigger.Request(),
    ).resolve_with_eh()

    node.sleep(Duration(seconds=4))

    AsyncServiceCall.create(
        node,
        node.close_gripper_client,
        Trigger.Request(),
    ).resolve_with_eh()


def retract() -> None:
    """Go to the home pose."""
    node = LLMActions()

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=node.home_pose),
    ).resolve_with_eh()


def move_gripper_by(x: float, y: float, z: float) -> None:
    """Move the gripper by some delta position."""
    # Get PoseStamped object of the end effector.
    node = LLMActions()

    eef_tf = node.get_tf("table", "eef")

    eef_tf.transform.translation.x += x
    eef_tf.transform.translation.y += y
    eef_tf.transform.translation.z += z

    target_pose = cast(PoseStamped, convert(eef_tf))

    AsyncServiceCall.create(
        node,
        node.move_eef_client,
        TargetPose.Request(target=target_pose),
    ).resolve_with_eh()
