## RGBDSLAM
RGBDSLAM is a RGB-D loop-closure graph-based SLAM approach using visual odometry and custom implementations of opencv methods

## Prequisites
This section **requires** the *catkin_ws* to be initialized and the *turtlebot_dabit* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)

## Installation
This process has to be done on __both__ _master_ and _turtlebot_ computers.  

Download rgbdslam_v2 and configure dependencies
1. CD to catkin_ws, source the environment setup, and download rgbdslam_v2
    * `cd ~/catkin_ws/src`
    * `git clone -b kinetic https://github.com/felixendres/rgbdslam_v2 rgbdslam`
2. Remove any conflicting libraries
    * `sudo apt remove ros-kinetic-libg2o libqglviewer-dev`
3. Install build dependencies
    * `sudo apt install cmake libeigen3-dev libsuitesparse-dev`
3. Build and install a custom verison of g2o
    * `cd ~/catkin_ws/src`
    * `git clone -b c++03 https://github.com/felixendres/g2o.git`
    * `mkdir g2o/build`
    * `cd g2o/build`
    * `cmake .. -DCMAKE_INSTALL_PREFIX=../install -DG2O_BUILD_EXAMPLES=OFF`
    * `nice make -j4 install`
      * If your build fails, try the `nice make -j4 install` command again
4. Install the library dependencies of rgbdslam_v2 using rosdep
    * `source ~/catkin_ws/devel/setup.sh`
    * `rosdep update`
    * `rosdep install rgbdslam`
5. Build rgbdslam
    * `export G2O_DIR=~/catkin_ws/src/g2o/install`
    * `cd ~/catkin_ws`
    * `catkin_make`

## Starting RGBDSLAM on the Turtlebot
On the _Turtlebot_ laptop:  
1. Open a new terminal
    1. `source ~/catkin_ws/devel/setup.sh`
    2. `export G2O_DIR=~/catkin_ws/src/g2o`
    3. `roslaunch rgbdslam headless.launch`

## Visualizing RGBDSLAM data
On the _master_ laptop, start the rgbdslam interface:
1. Open a new terminal
    1. `source ~/catkin_ws/devel/setup.sh`
    2. `rosrun rgbdslam rgbdslam`
2. Use the GUI to view the data from the rgbdslam node

## Tuning RGBDSLAM Parameters
Coming soon, we are working out the best parameters to use with this Turtlebot setup


## Troubleshooting
-

 

[Return to the main README page](/README.md)
