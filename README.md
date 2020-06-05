# My SLAM

:star2: __On going project :star2:__

A stereo visual SLAM package based on [slambook2](https://github.com/gaoxiang12/slambook2)
chapter 13. Ported to ROS for easier testing and usage.

## Data Structures

- Frame: a frame of stereo images
- Feature: 2D position of a feature on the image
- MapPoint: 3D position of a map point in the map
- Map: maintains the interface to the map; used by both frontend and backend
