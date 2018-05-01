#!/bin/bash
# Write our program to bash alias
CURRENT_DIR="$(dirname "${BASH_SOURCE[0]}")"
CURRENT_DIR=${CURRENT_DIR%/Setup/automated}

if [ "$CURRENT_DIR" == "." ]; then
  CURRENT_DIR=$CURRENT_DIR"/../.."
elif [ "$CURRENT_DIR" == "turtlebot2-tutorials" ]; then
  CURRENT_DIR="$(pwd)/turtlebot2-tutorials"
fi

if [ -z "$DABIT_DIR" ]; then
    echo "export DABIT_DIR=$CURRENT_DIR" >> ${HOME}/.bashrc
    DABIT_DIR="$CURRENT_DIR"
elif [ "$DABIT_DIR" != "$CURRENT_DIR" ]; then
    sed -i '1s!^!export DABIT_DIR='"$CURRENT_DIR"'\n!' ${HOME}/.bashrc
    echo $CURRENT_DIR" is different than "$DABIT_DIR
    echo "Setting DABIT_DIR environment variable to $CURRENT_DIR"
    DABIT_DIR="$CURRENT_DIR"
fi
export DABIT_DIR="$DABIT_DIR"

. "$DABIT_DIR/Setup/automated/environment.sh"
. "$DABIT_DIR/Setup/automated/definitions.sh"
. ${HOME}/.bashrc
echo ""
. dabit-setup-utility test
