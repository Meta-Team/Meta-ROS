#!/bin/bash
cd "$(dirname "$0")/.."
colcon build --symlink-install --packages-select serial
source ./install/setup.bash
colcon build --symlink-install --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'
