# TORCS control with ROS

## Installation
start by making a workspace. For example,

```Bash
mkdir torcs_ws

cd torcs_ws
```

Clone the repository into the ws

```Bash
git clone https://github.com/odedyec/ros-torcs-control
```

Take the installation file and move it to a folder where you want the simulator to be installed. For example your home, and run the installation script.

```Bash
cp ros-torcs-control/install_ros_torcs.sh ~/install_ros_torcs.sh; ~/install_ros_torcs.sh
```

## Compiling

compile once with ```catking_make ``` in the torcs_ws folder.

DON'T FORGET:

You should source in every terminal using ``` source PATH_TO_WORKSPACE/torcs_ws/devel/setup.bash ```

Easiest to add the line to ```~/.bashrc ```

