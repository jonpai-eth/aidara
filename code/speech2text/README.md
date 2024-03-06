# speech2text

A ROS 2 Node for speech-to-text conversion.

The node listens to the default microphone, waits for a keyword-prefixed instruction,
and publishes it on a topic.

To start the node, run

```bash
ros2 run speech2text speech2text
```
