#!/bin/bash

###
# Define definitions
###

. "$DABIT_DIR/Setup/automated/definitions.sh"

###
# Backup Files
###

__backup_workspace(){

  # The backup folder
  _backup_folder="$_backup_dir/old_workspace_"$(date '+%m-%d-%y_%H-%M-%S')

  if [ -d $_backup_folder ]; then
    echo "$_backup_folder ALREADY EXISTS! EXITING!"
    exit 0
  fi
  
  echo "Backing up the following folders to: $_backup_folder"

  # Make backup folder
  mkdir -p "$_backup_folder"

  # Copy or move files and folders to backup folder
  if [[ "$1" = "mv" ]]; then
    mv -v {"$_catkin_ws_dir","$_workspace_dir","$_tmp_dir"} "$_backup_folder"
  else
    cp -rv {"$_catkin_ws_dir","$_workspace_dir","$_tmp_dir"} "$_backup_folder"
  fi

  # Copy bashrc to backup folder
  cp -v {"$_bashrc_dir","$_rosrc_dir","$_aliases_dir"} "$_backup_folder"
}

confirm "Moving all working files to ~/old_workspace. Are you sure? [y/N]"
if [[ "$?" -eq 0 ]]; then
  __backup_workspace $1
fi
