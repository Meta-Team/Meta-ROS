# Meta-ROS

![humble](https://github.com/Meta-Team/Meta-ROS/actions/workflows/ros-humble.yml/badge.svg)

## Basic Info

Created by Yao Xinchen, 2023/7/13. \
Based on ROS2 Humble. \
Tested on Ubuntu 22.04. \
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
5. camera-info-manager

## Structure

4 main layers: Perception -> Decision -> Decomposition -> Execution

## Quickstart

### Upgrade CMake
Our project requires at least CMake 2.23 to compile, which is higher than the stock CMake version comes with Ubuntu 22.04. Please take a look at [Kitware APT Repository](https://apt.kitware.com/) to update your CMake version.

### Clone the Project
First, create a workspace and clone the project.

```Bash
mkdir -p meta_ws/src && cd meta_ws/src
git clone --recurse-submodules https://github.com/Meta-Team/Meta-ROS
```

### Install dependencies
This project is fully compatible with `rosdep`, you can easily install the dependencies with this command

```Bash
rosdep install -y --rosdistro humble --from-paths . --ignore-src
```

> **Note:** You may need to manually install Eigen3 because it cannot be automatically resolved by `rosdep`.

### Compile the Project
Simply run

```Bash
colcon build --symlink-install --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'
```
