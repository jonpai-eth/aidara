"""ROS 2 Node for converting text input to sound."""

import tempfile

import rclpy
from openai import OpenAI
from playsound import playsound
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class TextToSpeech(Node):
    """Text to speech node definition."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__("text_to_speech")

        self._client = OpenAI()
        self._skeleton_sub = self.create_subscription(
            String,
            "/user_feedback",
            self._speaking_callback,
            1,
            callback_group=ReentrantCallbackGroup(),
        )

    def _speaking_callback(self, string: String) -> None:
        """Outputs the text from the topic /user_feedback as speech."""
        response = self._client.audio.speech.create(
            model="tts-1",
            voice="echo",
            input=string.data,
        )
        with tempfile.NamedTemporaryFile(suffix=".mp3") as speech_file:
            response.write_to_file(speech_file.name)
            playsound(speech_file.name)
            speech_file.close()


def main(args: list[str] | None = None) -> None:
    """Initializes and runs the text to speech node."""
    rclpy.init(args=args)
    try:
        speech_node = TextToSpeech()

        executor = MultiThreadedExecutor()
        executor.add_node(speech_node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
