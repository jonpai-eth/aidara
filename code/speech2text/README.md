# speech2text

A ROS 2 Node for speech-to-text conversion.

The node listens to the default microphone, waits for a keyword-prefixed instruction,
and publishes it on a topic.

To start the node, run

```bash
ros2 run speech2text speech2text
```

## Parameters:

**energy_threshold** (Integer, default_value = 2000):

This parameter sets the energy threshold for audio input. It determines the ambient noise level that triggers the start of speech recognition.

- For noisy environments (2500-3500)
- For moderate noisy environments (2000-2500)
- For silent environments (1500-2000)

To change the Parameter run:

```bash
ros2 param set /speech2text energy_threshold <new_value_energy_threshold>
```
