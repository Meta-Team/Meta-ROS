# Meta-ROS

### Basic Info

Created by Yao Xinchen, 2023/7/13

Based on ROS2 foxy

Tested on Nvidia Orin Nano, Ubuntu 20.04

Used for RoboMaster Team Meta to control robots

### Dependencies

ROS2 foxy

### Structure

4 main layers: Perception, Decision, Decomposition, and Execution

Information is gotten by the Perception. Perception would transmit it to Decision, which would decide what behavior the robots should make. Decomposition then receive Decision's commands and decompose them to basic behaviors of hardwares. Finally, these information would be sent to the Execution and it would control the hardwares accordingly.

### To Compile

Build with ```colcon build --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'``` in the workspace

Update compile_commands with ```cp build/compile_commands.json compile_commands.json```