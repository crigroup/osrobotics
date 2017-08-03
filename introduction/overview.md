## Overview

Examples of real-world robotics applications, developed in our research group,
can be seen in this video:
{% youtube %} https://www.youtube.com/watch?v=zaOy3SdvXr4 {% endyoutube %}

Now, let us consider the following task: "pick up an object from a tabletop and
place it at another place".

![Starting and grasping configurations](../assets/grasping_before_after.png)

This task can be broken down into the following steps:

1. Determine the position of the object relative to the robot;
2. Move the robot gripper towards the object, without colliding with the
environment;
3. Grasp the object;
4. Move the grasped object towards the desired position, without colliding with
the environment;
5. Release the object.

Step 1 requires a vision sensor, typically a 3D camera, and *vision algorithms*
to find the position of the object with high enough precision. Such vision
algorithms will be covered in Chapter 4 (**Robot vision**) of this course.

Steps 2, 3 and 4 require generating the *commands* that move the robot gripper
to and from an appropriate position to grasp the object, without colliding with
the environment. This will be covered in Chapters 2 (**Manipulator kinematics**)
and Chapter 3 (**Motion planning**).

Putting together the different steps and the different hardware components
(vision sensor, robot arm, gripper, etc.) requires a versatile and robust
*software architecture*, which will be covered in Chapter 5 (**System**).

Some industrial applications, such as assembly, drilling, riveting, etc.,
require to precisely control the contact force between the robot and the
environment. This will be covered in Chapter 6 (**Force control**).
