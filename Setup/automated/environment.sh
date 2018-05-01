#!/bin/bash
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

echo "Password is needed to copy executable files"
sudo cp -r "$DABIT_DIR/Setup/automated/bin/." /usr/local/bin

cp -r "$DABIT_DIR/Setup/automated/ssh/." ${HOME}/.ssh
chown ${USER} "${HOME}/.ssh/turtlebot_rsa"
chown ${USER} "${HOME}/.ssh/turtlebot_rsa.pub"
chmod 600 "${HOME}/.ssh/turtlebot_rsa"
chmod 644 "${HOME}/.ssh/turtlebot_rsa.pub"
cat ${HOME}/.ssh/turtlebot_rsa.pub >> ${HOME}/.ssh/authorized_keys

cp -r "$DABIT_DIR/Setup/automated/Desktop/." ${HOME}/Desktop

cp "$DABIT_DIR/Setup/.rosrc" ${HOME}/.rosrc
cp ${HOME}/.bashrc ${HOME}/.bashrc_backup

remove_call(){
  sed -i 's/'"$1"'/#&/g' "$2"
}

sed -i -e '/-f ~\/.dabit_aliases/,+3d' ${HOME}/.bashrc
sed -i -e '/-f ~\/.rosrc/,+12d' ${HOME}/.bashrc
remove_call "source \/opt\/ros\/kinetic\/setup.bash" ${HOME}/.bashrc
remove_call "export TURTLEBOT_BASE=" ${HOME}/.bashrc
remove_call "export TURTLEBOT_3D_SENSOR=" ${HOME}/.bashrc
remove_call "export TURTLEBOT_STACK=" ${HOME}/.bashrc
remove_call "export ROS_MASTER_URI=" ${HOME}/.bashrc
remove_call "export ROS_IP=" ${HOME}/.bashrc
remove_call "export ROS_HOSTNAME=" ${HOME}/.bashrc
remove_call "export ROS_HOME=" ${HOME}/.bashrc

echo "
if [ -f ~/.rosrc ]; then 
    . ~/.rosrc
fi

if [ -f /opt/ros/kinetic/setup.bash ]; then 
    . /opt/ros/kinetic/setup.bash
fi

if [ -f ~/catkin_ws/devel/setup.bash ]; then 
    . ~/catkin_ws/devel/setup.bash
fi
" >> ${HOME}/.bashrc

eval "$(ssh-agent -s)"
ssh-add ~/.ssh/turtlebot_rsa
