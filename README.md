[![Build Status](https://travis-ci.com/vortexntnu/auv-cv.svg?branch=master)](https://travis-ci.com/vortexntnu/auv-cv)
# AUV-CV
The following directories are designed for AUV competition Robosub by Robonation. https://www.robonation.org/competition/robosub.

The taskes from 2018 can be found here: https://www.robonation.org/sites/default/files/2018%20RoboSub_2018%20Mission%20and%20Scoring_v01.50.pdf.

Clone it to the src folder of your workspace

## Dependencies 

Vortex_msgs: https://github.com/vortexntnu/vortex_msgs or https://github.com/vortexntnu/manta-auv

ROS-melodic: http://wiki.ros.org/melodic/Installation/Ubuntu

If you did not do the desktop-full installation of ros-melodic you might have to install the following packages manually
```sh
sudo apt install ros-melodic-camera-info-manager
sudo apt install ros-melodic-cv-bridge
sudo apt install ros-melodic-opencv3
sudo apt install ros-melodic-image-view
sudo apt install ros-melodic-image-transport

```

## ROS nodes

### Camera

#### Camera_front
Capture video from front camera using OpenCV and publish the video to the ROS topic `/camera/front`. 

Usage:
```bash
$ roslaunch camera camera_front.launch src:=[source]
```
Notice that there are multiple video sources (if no source is provided then the default camera at the computer is used).

Possible sources to be spesified for `[source]`:
- 0: video0
- 1: video1
- 2: video2
- 3: ~/Videos/GOPR1142.mp4
- 4: ~/Videos/guide_posts.mp4
- default: specify path to source

#### Camera_under
Capture video from camera under AUV using OpenCV and publish the video to the ROS topic `/camera/under`. 

Usage:
```bash
$ roslaunch camera camera_under.launch src:=[source]
```

### Video_stream_opencv
Package from ROS: http://wiki.ros.org/image_view.
Installation: https://github.com/ros-perception/image_pipeline.

Usage (for displaying video with image_view):
```bash
$ rosrun image_view image_view image:=<image topic> [image transport type]
```

### Pole_detect
This package have two nodes. The first is Task 1 of the competition. Uses color filtering together with contour detection by OpenCV to capture the pole(s) of the gate. 
Publishes the midpoint of the detected pole(s) on the topic: `pole_midpoint`.
Usage: 
```bash
$ roslaunch pole_detect gate_detect.launch topic:=[topic]
```
The second is detecting a pole and showing distance to it granted that the real-object size is known. 
Usage:
```bash
$ roslaunch pole_detect pole_detect.launch topic:=[topic]
```
**NB!** Theese will not work well without tuning the color parameters to your enviornment. Bruk rosrun rqt_reconfigure rqt_reconfigure, velg hvilken node og forandre HSV verdiene live. 

Topics should be spesified for `[topic]`. Any topic publishing a video may be spesified.
If no topic is provided, then pole_detect will subscribe to default topic of simulator: `/manta/manta/cameraunder/camera_image`.
Recomended topics from camera is `/camera/front` and `/camera/under`.

### Path_marker
Follows task 1 in the competition. The guide post points out the direction of the next obstacle/task. Uses color filtering together with line detection by OpenCV to capture the direction intended by the guide post. The direction (compared to the heading of the AUV) is buffered using the "sliding window"-method before returning the average.
The direction is published on the topic: `path_angle`.

Usage: 
```bash
$ roslaunch path_marker path_marker.launch topic:=[topic]
```
