# aidara

codebase containing all the code needed to run Alfred, an intelligent generalist assistant that works by giving a Large Language Model access to a robotic arm.

Developed by Timm Grigat, Pavidiran Ramanathan, Artur von Ruffer, Felix Hegg, Joel Kaufmann, Jakob Lüdke, Raman Besenfelder and Jonas Pai.

[\[Instagram\]](https://instagram.com/aidara.ethz)
[\[Youtube\]](https://www.youtube.com/@aidara_eth)
[\[LinkedIn\]](https://www.linkedin.com/company/99506275)
[\[website\]](https://aidara.ch)

Watch Timm and Jonas present this project at ETH Zurich (in german)

[![presentation](http://markdown-videos-api.jorgenkh.no/youtube/r_5VV-qFTWY)](https://youtu.be/r_5VV-qFTWY?t=444)

## How it works

Our setup includes a number of RGB cameras, a microphone, and a robot. Once the user gives an instruction, we send the instruction along with the current images to a LLM and instruct it to write a Python script fulfilling the request. To enable this, we created a small suite of APIs this script can make use of such as `move_gripper_to(pose: Pose)`. We then execute the script and profit.

## repository structure

```
.
├── code                  #   our code
│   ├── aidara_bringup        #   launch files
│   ├── aidara_common         #   shared utilities
│   ├── aidara_msgs           #   ROS 2 interface definitions
│   ├── camera_calibration    #   automated calibration of cameras
│   ├── geometric_grasp       #   naive grasp pose generation
│   ├── hand_position         #   hand tracking
│   ├── llm_planning          #   LLM API interfacing
│   ├── rerun_manager         #   rerun visualization
│   ├── speech_to_text        #   speech recognition
│   ├── staubli_controller    #   interface to a Stäubli TX2 robot
│   ├── text_to_speech        #   auditory feedback
│   ├── tf2_service           #   node offering tf2 functionality via services
│   ├── trajectory_planning   #   path planning using curobo
│   └── top_level_actions     #   API collection accessed by the LLM
└── dependencies          #   absolutely hacked forks
```

Consult the module READMEs for further information.

## Examples

### Alfred, give me something I can cut cables with

[![give cable cutter](http://markdown-videos-api.jorgenkh.no/youtube/XAxE92L_aPQ)](https://youtu.be/XAxE92L_aPQ)

### Alfred, I think I'm done here, can you help me clean up the desk?

[![clean up desk](http://markdown-videos-api.jorgenkh.no/youtube/LqWD9-XFnOw)](https://youtu.be/LqWD9-XFnOw)

### Alfred, do you see anything to turn yourself off with? Do it!

[![turn yourself off](http://markdown-videos-api.jorgenkh.no/youtube/KY6SK7pPxrs)](https://youtu.be/KY6SK7pPxrs)

### Alfred, put the green pen between the blue and yellow squares

[![put green pen](http://markdown-videos-api.jorgenkh.no/youtube/NXKs-Mg19ws)](https://youtu.be/NXKs-Mg19ws)

### Alfred, put the red pen on the red paper, the green pen on the green paper and the apple on the blue paper

[![sort by color](http://markdown-videos-api.jorgenkh.no/youtube/nVXTXjTeqOM)](https://youtu.be/nVXTXjTeqOM)

### Alfred, put all the food on the blue paper, all the writing utensils on the red paper and all the tools on the green paper

[![sort by category](http://markdown-videos-api.jorgenkh.no/youtube/0YvUgXeePtg)](https://youtu.be/0YvUgXeePtg)
