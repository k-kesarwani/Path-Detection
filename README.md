# Smart Path Planning using Raspberry Pi
## Introduction
The goal of this project is to implement a smart path planning algorithm for a drone using a Raspberry Pi. The algorithm uses the Rapidly-exploring Random Tree Star (RRT*) algorithm to plan an optimal path for the drone to follow. Additionally, the YOLOv7 algorithm is used for object detection, which enables the drone to avoid obstacles in its path.

## Prerequisites
Before starting the project, the following hardware and software are required:

Raspberry Pi
Drone
Camera module
YOLOv7 software
RRT algorithm software
Python programming language
Algorithm
The RRT algorithm is used to plan an optimal path for the drone to follow. The algorithm works by randomly exploring the search space and adding nodes to a tree. The tree is grown until a node is reached that is close to the destination. The path from the start node to the destination node is then extracted from the tree.

The YOLOv7 algorithm is used for object detection. The algorithm works by analyzing the input image from the camera module and identifying objects in the image. The drone uses this information to avoid obstacles in its path.

## Implementation
The project is implemented in Python. The YOLOv7 and RRT algorithms are integrated into a single program that runs on the Raspberry Pi. The program captures images from the camera module, runs the YOLOv7 algorithm to detect objects in the image, and then passes this information to the RRT algorithm to plan the path for the drone to follow.

## Results
The implementation of the smart path planning algorithm was successful. The drone was able to autonomously navigate through a maze of obstacles without any collisions. The YOLOv7 algorithm was effective in detecting obstacles in the drone's path, and the RRT algorithm was successful in planning a safe path for the drone to follow.

## Conclusion
In conclusion, the implementation of the smart path planning algorithm using a Raspberry Pi was successful. The algorithm was able to plan a safe and optimal path for the drone to follow while avoiding obstacles in its path. This project has potential applications in various industries, including agriculture, surveillance, and search and rescue.
