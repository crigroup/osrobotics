# Robot simulation in Gazebo

The usual pipeline to make a robot move is as follows
1. Plan a trajectory in OpenRAVE;
2. Convert the OpenRAVE trajectory to a ROS trajectory;
3. Send the ROS trajectory to the robot controller;
4. The robot controller moves the robot joints to the desired joint values at
the desired time instants, as specified by the trajectory, using its own
control algorithms, which are usually not accessible to the end user.

![Robot simulation/execution
 pipeline.](../assets/system/robot_simulation_pipeline.png)

Note that it is also possible to send joint values for one time instant at a
time. More details on this mode of operation will be given later.

In many situations, it is desirable to *simulate* the robot motion in software,
instead of executing it on the hardware. For that, in step 3 of the pipeline,
one can send the trajectory to a *simulated* controller in Gazebo. A key
implementation feature here is that the code to move the simulated robot and
that to move the real one should be the same. This is achieved by appropriate
code abstraction mechanisms.

The following example shows the full pipeline (with execution in Gazebo) for a
motion to grasp a cube on a table.
