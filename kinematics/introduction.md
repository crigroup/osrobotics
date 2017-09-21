# Manipulator kinematics

For the robot manipulator to perform a task (for example, grasping or
drilling), one needs to bring its *end-effector* to an appropriate
position/orientation.

-   For grasping, the end-effector is a gripper: one needs to bring the
    gripper to a position/orientation where the gripper can grasp the
    desired object;
-   For drilling, the end-effector is a drill bit: one needs to bring
    the drill bit towards the desired hole position, and the orientation
    of the drill bit must be perpendicular to the drilled surface.

![Bringing the drill bit to the desired position and orientation
   (perpendicular to the panel).](../assets/kinematics/drill.jpg)

As the robot commands are specified in terms of the manipulator *joint
angles*, one needs to find the *mapping* between the end-effector
position/orientation and the joint angles.

-   The process of finding the position/orientation of the end-effector
    given a set of joint angles is known as *forward kinematics*;
-   The process of finding the joint angles that realizes a given
    position/orientation of the end-effector is known as *inverse
    kinematics*.

Sections [Forward kinematics](forward_kinematics.md) and [Inverse
kinematics](inverse_kinematics.md) respectively presents these two
processes.