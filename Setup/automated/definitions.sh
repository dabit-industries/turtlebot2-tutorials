#!/bin/bash

###
# Define definitions
###

# Directory names
_backup="backup"
_catkin_ws="catkin_ws"
_workspace="workspace"
_tmp="tmp"
_dabit_dir="$DABIT_DIR"

# File Names
_rosrc=".rosrc"
_aliases=".dabit_aliases"
_bashrc=".bashrc"

# Where above directories and files are all located
_user_dir="$HOME"
_backup_dir="$_user_dir/$_backup"
_catkin_ws_dir="$_user_dir/$_catkin_ws"
_workspace_dir="$_user_dir/$_workspace"
_tmp_dir="$_user_dir/$_tmp"
_arduino_dir="$_user_dir/Arduino"
_pwd=$(pwd)

_rosrc_dir="$_user_dir/$_rosrc"
_aliases_dir="$_user_dir/$_aliases"
_bashrc_dir="$_user_dir/$_bashrc"

declare -a _apt_packages=("ros-kinetic-rosserial*" "python3-pyqt5" "libopencv*" "libcgal-dev" "libcgal-qt5*" "ros-kinetic-pointcloud-to-laserscan" "libpcap-dev" "libsuitesparse-dev" "ros-kinetic-imu-tools" "libavresample-dev" "libgphoto2-dev" "tesseract-ocr" "libtesseract-dev" "liblept5" "libleptonica-dev" "libopenni2-dev" "libopenni-dev" "libunicap2-dev" "libxine2-dev" "libgstreamer1.0-dev" "libgdal-dev" "libgdcm2-dev" "libudev-dev" "ros-kinetic-pointcloud-to-laserscan")
declare -a _remove_packages=()

# Which Dabit Packages to install
declare -a _ros_packages=("apriltags" "turtlebot_dabit" "turtlebot_dabit_description" "turtlebot_wanderer")
declare -a _workspace_packages=("apriltags_cpp")
_opencv_install_args="-DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=$HOME/workspace/opencv_contrib-3.2.0/modules -DTesseract_INCLUDE_DIR=/usr/include/tesseract"
declare -a _workspace_install_packages=("apriltags_cpp")


# Miscelleneous Defines
_arduino_tar="arduino-1.8.2-linux64.tar.xz"
