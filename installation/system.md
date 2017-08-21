# System: ROS, Gazebo

The Robot Operating System (ROS) is a flexible framework for writing robot
software. It is a collection of tools, libraries, and conventions that aim to
simplify the task of creating complex and robust robot behavior across a wide
variety of robotic platforms.

## ROS installation

Set-up your computer to accept software from packages.ros.org and set-up your
keys following the steps described at http://wiki.ros.org/kinetic/Installation/Ubuntu

Now, install the ROS bare bones
{% label %}command-line{% endlabel %}
```bash
# Installation
sudo apt-get update
sudo apt-get install python-wstool ros-kinetic-ros-base
# Initialize rosdep
sudo rosdep init
rosdep update
```

### Create a ROS workspace

Let’s create a [catkin workspace](http://wiki.ros.org/catkin/workspaces)
{% label %}command-line{% endlabel %}
```bash
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws/src
catkin_init_workspace
```

Even though the workspace is empty (there are no packages in the `src` folder,
just a single `CMakeLists.txt` symbolic link) you can still build the workspace
{% label %}command-line{% endlabel %}
```bash
cd ~/catkin_ws/
catkin_make
```

Before continuing, source your workspace `setup.bash` file
{% label %}command-line{% endlabel %}
```bash
source ~/catkin_ws/devel/setup.bash
```

To make sure your workspace is properly overlayed by the setup script, make
sure `ROS_PACKAGE_PATH` environment variable includes the path to your
workspace
{% label %}command-line{% endlabel %}
```bash
echo $ROS_PACKAGE_PATH
```

This is the expected output
```
/home/USERNAME/catkin_ws/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks
```

### Configure your .bashrc

Every time you start a new terminal, the file `~/.bashrc` will be sourced. When
you already have an opened terminal, you can source it manually running this
command
{% label %}command-line{% endlabel %}
```bash
source ~/.bashrc
```

In order to use ROS, you need to source the ROS `setup.bash` file, therefore,
it's recommended to include the previous command in your `.bashrc` file.
You can do so by running this
{% label %}command-line{% endlabel %}
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Testing the installation

Let's try to run `roscore` in a first terminal. Then, *publish* a `std_msgs/String`
message to the topic `/some_topic_name` a second terminal. Finally, *subscribe*
to the same topic and print it out the message in a third terminal:
{% label %}command-line{% endlabel %}
```bash
# Terminal 1
roscore
# Terminal 2
rostopic pub -r 1 /some_topic_name std_msgs/String 'Hello World!'
# Terminal 3
rostopic echo /some_topic_name
```

### ROS packages installation
For the sections where interaction with hardware components is required,
we will simulate these components using Gazebo. More details can be found in
the [ROS architecture section](../system/architecture.md).

### ROS packages from source

Go to the source folder of your ROS workspace
{% label %}command-line{% endlabel %}
```bash
cd ~/catkin_ws/src
```

Make sure that you have [cloned the course repository](../installation/basic_tools.md#git).

Now, use the `wstool` to install required packages from source repositories
{% label %}command-line{% endlabel %}
```bash
wstool init .
wstool merge osr_course_pkgs/dependencies.rosinstall
wstool update
```

Install any missing dependencies using `rosdep`
{% label %}command-line{% endlabel %}
```bash
rosdep update
rosdep install --from-paths . --ignore-src -y
```

Now compile your ROS workspace:
{% label %}command-line{% endlabel %}
```bash
cd ~/catkin_ws && catkin_make install
```

### Dependencies

`rosdep` is very useful to install system dependencies but those dependencies
have to be included by the ROS community, therefore few packages need to be
install manually:
{% label %}command-line{% endlabel %}
```bash
# Packages from Ubuntu repositories
sudo apt-get install blender openscad python-rtree
# Python modules
pip install control trimesh --user
```

### Testing the installation

Let's try visualizing the robot using [RVIZ](wiki.ros.org/rviz)
{% label %}command-line{% endlabel %}
```bash
roslaunch osr_description visualize_robot_gripper.launch
```

![Robot visualization in RVIZ.](../assets/installation/denso_in_rviz.png)

Play with the sliders to see how you can move the robot. This is a simple
visualization where nothing is actually simulated. A simple test could be to
move the robot to a configuration where it is in self-collision.

Finally, check that you can run the gazebo simulation:
{% label %}command-line{% endlabel %}
```bash
roslaunch osr_gazebo robotic_setup.launch
```

> **Note** The first time you run gazebo, it will download several models
available online. You may want to start first gazebo alone and give it time to
download the models:
>
> `gzserver --verbose`

![Robot simulation in Gazebo.](../assets/installation/denso_in_gazebo.jpg)

> **Warning** In case of failure starting the simulation, please check the
[system requirements for Gazebo](http://gazebosim.org/tutorials?tut=guided_b1&cat=#Systemrequirements).
If your computer doesn't have a dedicated video card, it's likely that you
won't be able to run the graphical interface. Try again setting the `gui`
parameter to false
>
> `roslaunch osr_gazebo robotic_setup.launch gui:=false`

## To learn more about this topic

Take a look at the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials).
