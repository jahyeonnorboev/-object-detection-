ROS2 Humble based Smart Office Assistant Robot.
 Voice Control (Korean / English)
- Human Registration
- Human Following using YOLOv8
- LiDAR Obstacle Avoidance
- Human Reacquisition

   ## Main Files

# voice_robot_control.py
Voice recognition and command processing.

# last_following.py
YOLO + LiDAR based human following.

## Environment

- Ubuntu 22.04
- ROS2 Humble
- TurtleBot3
- YOLOv8
- OpenCV
- Vosk Speech Recognition

## Run

python3 voice_robot_control.py

python3 last_following.py

System automatically launches:

- TurtleBot3 Bringup
- LiDAR
- Camera
- Human Following
- Voice Control
- rosbridge
- Web Interface
```
#      Web Interface
turtlebot3_control_v3.2.html
roslib.min.js
eventemitter2.min.js
#   ****** Run *****
chmod +x start_robot.sh
./start_royce.sh

This launches:

TurtleBot3 Bringup
LiDAR
Camera
Human Following
Voice Control
rosbridge
Web Interface
