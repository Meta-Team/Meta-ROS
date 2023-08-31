# Meta-ROS

### Basic Info

Created by Yao Xinchen, 2023/7/13

Based on ROS2 foxy

Tested on Nvidia Orin Nano, Ubuntu 20.04

Used for RoboMaster Team Meta to control robots

### Dependencies

ROS2 foxy

### Structure

4 main layers: Perception -> Decision -> Decomposition -> Execution

### To Compile

Build with ```colcon build --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'``` in the workspace

Update ROS extension with ctrl+shift+p -> ROS: Update C++ Properties -> ROS: Update Python Path

Update clangd with ctrl+shift+p -> clangd: Restart language server

If clangd is not activated, use ```cp build/compile_commands.json compile_commands.json```