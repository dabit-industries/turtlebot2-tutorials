#!/bin/bash

###
# Define definitions
###

. "$DABIT_DIR/Setup/automated/definitions.sh"

###
# Backup Files
###

. "$_dabit_dir/Setup/automated/backup.sh" "mv"

###
# Workspace Setup
###

__setup_workspace(){
  # Set up new workspace
  mkdir -p "$_catkin_ws_dir"
  mkdir -p "$_workspace_dir"
  mkdir -p "$_tmp_dir"
}
echo "Setting up new workspace"
__setup_workspace

###
# Catkin Setup
###

__make_catkin_workspace(){
  # Set up catkin workspace
  mkdir -p "$_catkin_ws_dir/src"
  catkin_make --directory "$_catkin_ws_dir"
}

echo "Making new catkin workspace"
__make_catkin_workspace

###
# Copy Packages
###

__copy_catkin_ws(){
  for i in "${_ros_packages[@]}"
  do
    cp -rv "$_dabit_dir/Setup/catkin_ws/src/$i" "$_catkin_ws_dir/src"
  done
  for i in "${_workspace_packages[@]}"
  do
    cp -rv "$_dabit_dir/Setup/workspace/$i" "$_workspace_dir"
  done
}

echo "Copying ROS packages"
__copy_catkin_ws

###
# Install Dependencies
###

__install_dependencies(){
  echo "Sudo is needed for installing packages"
  sudo apt install ${_apt_packages[@]} -y
}

echo "Installing dependencies"
__install_dependencies

###
# Install Arduino IDE
###

__install_arduino_ide(){
  arduino_folder = ${_arduino_tar%-linux64.tar.xz}
  if [ -z "$_user_dir/$arduino_folder" ]; then
    wget "https://downloads.arduino.cc/$_arduino_tar" -P "$_tmp_dir"
    cd "$_tmp_dir"
    tar -xvf "$_tmp_dir/$_arduino_tar" -C "$_user_dir/$arduino_folder"
    cd "$_user_dir/arduino*"
    . install.sh
    mkdir -p "$_user_dir/Arduino/libraries"
    cd "$_user_dir/Arduino/libraries"
    rosrun rosserial_arduino make_libraries.py .
    sudo usermod -a -G dialout turtlebot
  fi
  mkdir -p "$_user_dir/Arduino"
  cp -r "$_dabit_dir/Setup/Arduino/." "$_user_dir/Arduino"
}

echo "Installing Arduino IDE"
__install_arduino_ide

###
# Build Packages
###

__build_ws(){
  # Build workspace stuff
  for i in "${_workspace_install_packages[@]}"
  do
    cd "$_workspace_dir/$i"
    mkdir "build"
    cd "build"
    cmake ".."
    make
    sudo make install
  done
   
  catkin_make --directory "$_catkin_ws_dir"
}

echo "Building Workspace"
__build_ws


###
# Running Test Code
###

__run_checks(){
  echo "Checks not implemented yet"
}

echo "Running Checks and Tests"
__run_checks

cd $_PWD
. ~/.bashrc
echo "Installation Complete"
