# Meta-ROS

## Basic Info

Created by Yao Xinchen, 2023/7/13. \
Based on ROS2 Humble. \
Tested on Nvidia Orin Nano, Ubuntu 22.04. \
Used for RoboMaster Team Meta to control robots.

This project is hoped to be modular, readable and easy to maintain.

## Doc

There's a document describing how this project is designed -> 
[Meta-ROS-Doc Web](https://yao-xinchen.github.io/Meta-ROS-Doc/meta-ros.html)

Its source code is provided here ->
[Meta-ROS-Doc Source](https://github.com/Yao-Xinchen/Meta-ROS-Doc)

## Dependencies

1. ros-humble
2. serial-driver
3. moveit
4. moveit-servo

## Structure

4 main layers: Perception -> Decision -> Decomposition -> Execution

## To Compile

First, create a workspace and clone the project.

```Bash
mkdir Meta-ROS
cd Meta-ROS
git clone --recurse-submodules https://github.com/Yao-Xinchen/Meta-ROS src
```

To build run the script `first_build.bash` or `first_build.zsh` with

```Shell
# For bash
bash src/first_build.bash
# For zsh
zsh src/first_build.zsh
```

or run the following commands:

```Bash
colcon build --symlink-install --packages-select serial
source ./install/setup.bash
colcon build --symlink-install --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'
```
