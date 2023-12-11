"""
ROS 2 Node for speech-to-text conversion.

The node listens to the default microphone, waits for a keyword-prefixed instruction,
and publishes it on a topic.
"""
import string

import rclpy
import sounddevice  # noqa: F401
import speech_recognition as sr
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class Speech2Text(Node):
    """ROS 2 Node for speech-2-text conversion using OpenAI Whisper."""

    def __init__(self) -> None:
        """Initialize Speech2Text."""
        super().__init__("speech2text")

        self._keyword = "alfred"
        self._recognizer = sr.Recognizer()

        self._punctuation_replacement = str.maketrans("", "", string.punctuation)

        self.declare_parameter("energy_threshold", 2000)
        self._recognizer.energy_threshold = (
            self.get_parameter("energy_threshold").get_parameter_value().integer_value
        )
        self.add_on_set_parameters_callback(self._parameter_callback)

        self._pub = self.create_publisher(
            String,
            "speech2text",
            1,
        )

    def _parameter_callback(
        self,
        parameters: list[Parameter],
    ) -> SetParametersResult:
        """Forward energy_threshold parameter change to the recognizer."""
        new_energy_threshold = next(
            filter(lambda param: param.name == "energy_threshold", parameters),
            None,
        )
        if new_energy_threshold:
            if not new_energy_threshold.type_ == rclpy.Parameter.Type.INTEGER:
                raise ValueError(new_energy_threshold)
            self._recognizer.energy_threshold = new_energy_threshold.value
            self.get_logger().info(
                f"Updated energy_threshold to {self._recognizer.energy_threshold}.",
            )
        return SetParametersResult(successful=True)

    def _transcribe_microphone(self) -> str | None:
        """Transcribe microphone input using OpenAI Whisper."""
        with sr.Microphone() as microphone:
            audio_listened = self._recognizer.listen(microphone)

        if not audio_listened.frame_data:
            return None

        try:
            text = self._recognizer.recognize_whisper(
                audio_listened,
                model="large-v3",
                language="english",
            )
        except sr.UnknownValueError:
            self.get_logger().info("speech2text was unable to understand a phrase.")
            return None

        stripped_text = text.strip().lower().translate(self._punctuation_replacement)
        self.get_logger().debug(f"recognized text: '{stripped_text}'.")

        return stripped_text

    def _extract_instruction(self, stripped_text: str) -> str | None:
        """Extract instruction from text."""
        _preamble, *instructions = stripped_text.split(self._keyword)

        if not instructions:
            return None

        if len(instructions) > 1:
            self.get_logger().error(
                "Cannot handle multiple instructions simultaneously."
                f" Please use '{self._keyword}' only once at a time.",
            )
            return None

        return instructions[0].strip()

    def listen(self) -> None:
        """Extract an instruction from the microphone and publish it on /speech2text."""
        stripped_text = self._transcribe_microphone()
        if not stripped_text:
            return

        extracted_instruction = self._extract_instruction(stripped_text)
        if not extracted_instruction:
            return

        msg = String()
        msg.data = extracted_instruction
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Initialize and run the Speech2Text node."""
    rclpy.init(args=args)

    try:
        speech2text_node = Speech2Text()
        executor = MultiThreadedExecutor()
        executor.add_node(speech2text_node)

        while rclpy.ok():
            listen = executor.create_task(speech2text_node.listen)
            executor.spin_until_future_complete(listen)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
