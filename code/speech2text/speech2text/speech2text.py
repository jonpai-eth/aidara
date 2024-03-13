"""
ROS 2 Node for speech-to-text conversion.

The node listens to the default microphone, waits for a keyword-prefixed instruction,
and publishes it on a topic.
"""
import string

import rclpy
import sounddevice  # noqa: F401
import speech_recognition as sr
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class Speech2Text(Node):
    """ROS 2 Node for speech-2-text conversion using OpenAI Whisper."""

    def __init__(self) -> None:
        """Initialize Speech2Text."""
        super().__init__("speech2text")

        self._keyword = "can you"
        self._recognizer = sr.Recognizer()

        self._min_energy_threshold = 800
        self._max_energy_threshold = 3000
        self._recognizer.energy_threshold = 800
        self._recognizer.dynamic_energy_threshold = False

        self._counter = 0
        self._max_counter = 5

        self._punctuation_replacement = str.maketrans("", "", string.punctuation)

        self._pub = self.create_publisher(
            String,
            "/speech2text",
            1,
        )

    def _transcribe_microphone(self) -> str | None:
        """Transcribe microphone input using OpenAI Whisper."""
        with sr.Microphone() as microphone:
            audio_listened = self._recognizer.listen(microphone)
            self._counter += 1


            if not audio_listened.frame_data:
                return None

            if self._counter >= self._max_counter:
                self.get_logger().info("Adjusting to ambient noise...")
                self._recognizer.adjust_for_ambient_noise(microphone)
                self._counter = 0
                self._check_ambient_noise_adjustment()

        try:
            text = self._recognizer.recognize_whisper(
                audio_listened,
                model="large-v3",
                language="english",
            )
            self.get_logger().info(text)
        except sr.UnknownValueError:
            self.get_logger().info("speech2text was unable to understand a phrase.")
            return None

        stripped_text = text.strip().lower().translate(self._punctuation_replacement)
        self.get_logger().debug(f"recognized text: '{stripped_text}'.")

        return stripped_text

    def _check_ambient_noise_adjustment(self) -> None:
        """Check if energy_threshold needs to be adjusted within a range (800-3000)."""
        if self._recognizer.energy_threshold < self._min_energy_threshold:
            self._recognizer.energy_threshold = self._min_energy_threshold
            self.get_logger().info(
                f"Adjustment lower than Min.: {self._recognizer.energy_threshold}")

        elif self._recognizer.energy_threshold > self._max_energy_threshold:
            self._recognizer.energy_threshold = self._max_energy_threshold
            self.get_logger().info(
                f"Adjustment higher than Max.: {self._recognizer.energy_threshold}")

        else:
            self.get_logger().info(
                f"New energy_threshold: {self._recognizer.energy_threshold}")


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
        self.get_logger().info("Listening...")
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
