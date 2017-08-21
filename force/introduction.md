## Introduction to force control
The purpose of the previous chapters was to make the robot move around without touching the environment (collision-free motions). If the robot needed to touch an object in the environment, the contact would be a simple one, such as in grasping. In some industrial applications, such as assembly, grinding, drilling, etc. it is however necessary to come to contact with the environment and to control the complex interaction forces that arise from the contact between the robot and the environment.

Consider for instance the task of sliding a pin on the surface of a woodblock prior to its insertion (Fig. 1).

![Pin insertion](../assets/pin_insertion.png)
Fig. 1: Sliding a pin on the surface of a woodblock.

To maintain the contact between the pin and the woodblock, the normal contact force (red arrow) need to be always strictly positive. However, if the normal contact force is too high, it will, because of friction, generate high tangential contact forces, which in turn will make the sliding motion (green arrow) difficult. Therefore, it is necessary to control the normal contact force to be within a range of 1 to 5 Newtons. How to reliably achieve such a control is a purpose of this chapter.

## Force control with position-controlled robots

Most industrial robots are controlled in position, that is, one can specify desired joint angles – as opposed to, e.g., desired joint torques, which are directly related to the force exerted by the robot. Internally, the robot controller will translate the desired joint angles into appropriate commands, e.g. motor currents. Such low-level commands are generally not accessible to the end users.

To overcome this limitation, one can augment position-controlled robots with Force/Torque (F/T) sensors mounted near the end-effector (Fig. 1). Then, by closing a feedback loop on top of the measures provided by the F/T sensors, it is possible to implement force control, as shown in Section [Principles of force control](../force/principles.md)

Using these principles, one can next implement a variety of force control schemes, as detailed in Section [Examples: hybrid control and impedance control.](../force/examples.md)
