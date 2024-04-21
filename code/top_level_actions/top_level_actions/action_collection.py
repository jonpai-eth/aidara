"""Collection of top level actions for the LLM Planner."""

from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .llm_actions_node import LLMActions
from aidara_common.tf_utils import convert
from aidara_msgs.srv import GeometricGrasp, TargetPose, Tf2GetTransform


def say(message: str) -> None:
    """Relay a message to the user."""
    msg = String(data=message)
    LLMActions().user_feedback_pub.publish(msg)


def pick_object_from_table(object_description: str) -> None:
    """Pick an object from the table."""
    node = LLMActions()

    geometric_grasp_req = GeometricGrasp.Request()
    geometric_grasp_req.object_description.data = object_description
    grasp_pose = node.call_with_eh(
        node.geometric_grasp_client,
        geometric_grasp_req,
    ).pose

    node.call_with_eh(node.open_gripper_client, Trigger.Request())

    node.call_with_eh(node.move_eef_client, TargetPose.Request(target=grasp_pose))

    node.call_with_eh(node.close_gripper_client, Trigger.Request())

    node.home()


def place_object_on_table_at(x: float, y: float) -> None:
    """Place on object on the table."""
    node = LLMActions()

    target_pose = PoseStamped()
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.header.frame_id = "table"
    target_pose.header.stamp = node.get_clock().now().to_msg()

    node.call_with_eh(node.move_eef_client, TargetPose.Request(target=target_pose))

    node.call_with_eh(node.open_gripper_client, Trigger.Request())

    node.home()


def take_from_hand() -> None:
    """Take an object from the user's hand."""
    node = LLMActions()

    handover_pose = node.get_handover_pose()

    node.call_with_eh(node.move_eef_client, TargetPose.Request(target=handover_pose))

    node.call_with_eh(node.open_gripper_client, Trigger.Request())

    node.sleep(Duration(seconds=4))

    node.call_with_eh(node.close_gripper_client, Trigger.Request())

    node.home()


def give_to_hand() -> None:
    """Give an object to the user's hand."""
    node = LLMActions()

    handover_pose = node.get_handover_pose()

    node.call_with_eh(node.move_eef_client, TargetPose.Request(target=handover_pose))

    node.call_with_eh(node.open_gripper_client, Trigger.Request())

    node.sleep(Duration(seconds=4))

    node.call_with_eh(node.close_gripper_client, Trigger.Request())

    node.home()


def move_gripper_by(x: float, y: float, z: float) -> None:
    """Move the gripper by some delta position."""
    # Get PoseStamped object of the end effector.
    node = LLMActions()

    transform_req = Tf2GetTransform.Request(target_frame="eef", source_frame="table")
    eef_tf: TransformStamped = node.call_with_eh(
        node.tf_get_transform_client,
        transform_req,
    ).tf

    eef_tf.transform.translation.x += x
    eef_tf.transform.translation.y += y
    eef_tf.transform.translation.z += z

    target_pose = convert(eef_tf)
    node.call_with_eh(node.move_eef_client, TargetPose.Request(target=target_pose))
