# Open-Source Robotics Book

## Presentation

The objective of this course is to equip the reader with the *scientific and
technical* knowledge to develop advanced applications in industrial robotics.
The main features of this course are:

1. Because robotics implies by essence the development of robotic *systems*,
every theoretical notions we introduce will be accompanied by practical software
examples and exercises. Our objective is that, by the end of this course, the
reader will be able to use and/or develop advanced real-world robotics
applications by herself. Examples of such applications, developed in our
research group, can be seen in the video below.

{% youtube src="https://www.youtube.com/watch?v=zaOy3SdvXr4" %}{% endyoutube %}

2. Conversely, we only introduce the theoretical notions that are useful for
industrial applications and relevant with regards to the intended open-source
software. As a consequence, on many occasions, we shall refer the interested
reader to appropriate articles or textbooks for a more complete coverage.

3. This course is completely free, and so is all the required software, as we
shall use only (state-of-the-art) open-source software, such as ROS, OpenRAVE,
OpenCV, PCL, etc. Hardware is emulated through Gazebo, an open-source,
high-quality, physics simulator, such that there is no need to purchase any
hardware.

## Overview

Consider the following task: "pick up an object from a tabletop and place it at
another place".

This task can be broken down into the following steps:

1. Determine the position of the object relative to the robot;
2. Move the robot gripper towards the object, without colliding with the
environment;
3. Grasp the object;
4. Move the grasped object towards the desired position, without colliding with
the environment;
5. Release the object.
