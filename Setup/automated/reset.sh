#!/bin/bash

###
# Define definitions
###

. "$DABIT_DIR/Setup/automated/definitions.sh"

###
# Reset Files
###

confirm "Resetting Workspace. Are you sure? [y/N]"
if [[ "$?" -eq 0 ]]; then
  . "$_dabit_dir/Setup/automated/backup.sh" "mv"
  . "$_dabit_dir/Setup/automated/automate.sh"
  rm ~/.config/ros.org/rqt_gui.ini
  rm ~/.config/Trolltech.conf
fi


echo "RESET COMPLETE"
