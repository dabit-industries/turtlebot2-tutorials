#!/bin/bash

###
# Define definitions
###

. "$DABIT_DIR/Setup/automated/definitions.sh"

###
# Update Files
###

__update_files(){
  cd "$_dabit_dir"
  if [ ! -d "$_dabit_dir/.git" ]; then
    echo "Not a git repository"
    return 1
  fi
  _git_status=$(git pull | grep "up-to-date")
  if [ -z "$_git_status" ]; then
    echo "Updates found."
  else
    echo "No updates found."
  fi

  echo "Updating not fully written."
  echo "Please run the following commands:"
  echo "  dabit-setup-utility reset"
  echo "  dabit-setup-utility install"
}

confirm "Updating all files. If updates are found, all exising code is moved, and the new code is installed.
Are you sure? [y/N]"
if [[ "$?" -eq 0 ]]; then
  __update_files
fi

cd "$_pwd"
echo "UPDATE COMPLETE"
