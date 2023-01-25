#!/bin/bash

# Machine setup (Ubuntu)
# Python3 venv: needed to create virtual environments
# cmake, ninja, g++: needed for building opencv-python and pupil_apriltags packages
# v4l-*: useful for camera
sudo apt install python3.10-venv git ninja-build cmake g++ v4l-conf v4l-utils

# Virtual environment creation
python3 -m venv ~/Visionvenv --upgrade-deps
source ~/Visionvenv/bin/activate

# Virtual environment package installation
# requires: cmake, ninja, g++
pip3 install numpy pupil_apriltags opencv-python
