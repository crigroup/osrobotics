{% include "../math.md" %}

# Robot vision assignment

Prerequisite: Chapter 4.

> #### Exercise::Finding the pose of a planar glass
>
First, make sure that you have
[cloned the course repository](../installation/basic_tools.md#git). In
the folder
$$\texttt{~/catkin_ws/src/osr_course_pkgs/osr_examples/vision_assignment_data}$$ 
you can find the following files  (n = 1, 2, 3, 4):
- $$\texttt{take_n.ply}$$: contains the point cloud of take n;
- $$\texttt{take_n_left.png}$$: contains the left camera image for take n;
- $$\texttt{take_n_right.png}$$: contains the right camera image for
  take n;
- $$\texttt{left_camera_info.yaml}$$: left camera info;
- $$\texttt{right_camera_info.yaml}$$: right camera info.
>
>
![Left camera image for take 1.](../assets/assignments/take_1_left.png)
>
Assume that the pose of the camera is fixed across the four
takes. Write the code to determine the relative
transformations $$T_2$$, $$T_3$$, $$T_4$$ between the poses of the
glass in takes 2, 3, and 4, respectively, and the pose of the glass in take 1.



