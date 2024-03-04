#!/bin/zsh
cd "$(dirname "$0")/.."
colcon build --symlink-install --packages-select serial
source ./install/setup.zsh
colcon build --symlink-install --cmake-args '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'
