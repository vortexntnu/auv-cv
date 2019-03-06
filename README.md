# AUV-CV
Download this repo as a subfolder to src in catkin workspace of ROS. 

## Installing dependencies
OpenCV >= 3.4.0: https://github.com/opencv/opencv
ROS - Kinetic Kame: http://wiki.ros.org/kinetic

```sh
sudo apt install ros-kinetic-camera-info-manager
sudp apt install ros-kinetic-cv-bridge
```

## Competition
The following directories are designed for AUV competition Robosub by Robonation. https://www.robonation.org/competition/robosub.

The taskes from 2018 can be found here: https://www.robonation.org/sites/default/files/2018%20RoboSub_2018%20Mission%20and%20Scoring_v01.50.pdf.

## Content

### Camera_front
Capture video from front camera using OpenCV and publish the video to the ROS topic '/camera/front'. 
Usage:
```sh
rosrun video_front video_front [source]
```
Notice that there are multiple video sources (if no source is provided then the default camera at the computer is used).
- 0: video0
- 1: video1
- 2: video2
- 3: GOPR1142.mp4 @ ~/Videos
- 4: guide_posts.mp4 @ ~/Videos
- default: specify path to source

### Camera_under
Will be added later.
Capture video from front camera using OpenCV and publish the video to the ROS topic '/camera/under'. 
Usage:
```sh
rosrun video_under video_under [source]
```

### Video_stream_opencv
Package from ROS: http://wiki.ros.org/image_view.
Installation: https://github.com/ros-perception/image_pipeline.
Usage (for displaying video with image_view):
```sh
rosrun image_view image_view image:=<image topic> [image transport type]
```

### Pole_detect
Task 1 of the competition. Uses color filtering together with contour detection by OpenCV to capture the pole(s) of the gate. 
Publishes the midpoint of the detected pole(s) on the topic: 'pole_midpoint'.
Usage: 
```sh
rosrun gate_detect gate_detect
```

### Path_marker
Follows task 1 in the competition. The guide post points out the direction of the next obstacle/task. Uses color filtering together with line detection by OpenCV to capture the direction intended by the guide post. The direction (compared to the heading of the AUV) is buffered using the "sliding window"-method.
The direction is published on the topic: 'path_angle'.
Usage: 
```sh
rosrun guide_post guide_post
```