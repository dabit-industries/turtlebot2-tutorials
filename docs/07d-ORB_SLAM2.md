## ORB SLAM 2


## Installation
Download orb_slam2 and configure dependencies
1. CD to workspace and download orb_slam2
    * `cd ~/workspace``
    * `git clone https://github.com/raulmur/ORB_SLAM2``
2. Make a temporary directory in the workspace
    * `mkdir ~/workspace/tmp`
3. Build and Install Intel LibRealSense
    * `cd ~/workspace/tmp`
    * `git clone https://github.com/IntelRealSense/librealsense`
    * `cd librealsense`
    * `mkdir build`
    * `cd build`
    * `cmake ..`
    * `make`
    * `sudo make install`
4. Build and Install Pangolin
    * `cd ~/workspace/tmp`
    * `git clone https://github.com/stevenlovegrove/Pangolin.git`
    * `git checkout v0.5`
    * `sudo apt install libglew-dev`
    * `cd Pangolin`
    * `mkdir build`
    * `cd build`
    * `cmake ..`
    * `make`
    * `sudo make install`
5. Build ORB_SLAM2 base libraries
    * `cd ~/workspace/ORB_SLAM2`
    * `chmod +x build.sh`
    * `./build.sh`
6. Build ORB_SLAM2 for ROS
    * `export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/workspace/ORB_SLAM2/Examples/ROS`
    * `chmod +x build_ros.sh`
    * `./build_ros.sh`
7. Run ORB_SLAM2 RGBD Example
    * `rosrun ORB_SLAM2 RGBD ~/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/workspace/ORB_SLAM2/Examples/RGB-D/TUM1.yaml`

## Starting ORB_SLAM2 on the Turtlebot
ORB_SLAM2 ROS support is lacking (a lot)

## Visualizing ORB_SLAM data
No support yet

## Viewing ORB_SLAM data
No support yet

## Tuning ORB_SLAM parameters
No support yet

## Troubleshooting


 

[Return to the main README page](/README.md)
