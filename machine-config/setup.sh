#!/bin/bash

scriptdir=$(dirname $(realpath $0))
venvname="Visionvenv"

# Machine setup (Ubuntu)
# Python3 venv: needed to create virtual environments
# cmake, ninja, g++: needed for building opencv-python and pupil_apriltags packages
# v4l-*: useful for camera
sudo apt install python3.10-venv git ninja-build cmake g++ v4l-conf v4l-utils

# Virtual environment creation
python3 -m venv ~/${venvname} --upgrade-deps
source ~/${venvname}/bin/activate

# Virtual environment package installation
# requires: cmake, ninja, g++
pip3 install -r ${scriptdir}/requirements.txt

echo "Required Python tools have been installed in a virtual environment (venv) at: "
echo " -> ~/${venvname} (aka $HOME/${venvname})"
echo ""
echo "To activate venv before executing any scripts, perform the following in your shell"
echo " source ~/${venvname}/bin/activate"
