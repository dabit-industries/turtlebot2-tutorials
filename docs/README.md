# Turtlebot Tutorials

NOTE: These tutorials are currently being revamped. They have not been checked for sleeping children.

## Cloning this repository
Please use the following command to clone this repo:
```console
git clone --recursive git@github.com:dabit-industries/turtlebot2-tutorials ~/turtlebot2-tutorials
```

## Automated Setup
Bypass the environment setup instructions in the tutorial with the [Automated Setup](00b-Automated_Setup.md)  
Checkout the [Turtlebot Code and Setup Files](/Setup)  

## Tutorials
ROS 
* [ROS Setup](02-Master_Setup.md)

* [Prevent Laptop Locking/Sleeping](01b-Turtlebot_Ubuntu_Setup.md)

* [ROS Master/Turtlebot Computer Network Setup](02b-Network_Setup.md)
  * [Tips and Tricks](02c-Tips_and_Tricks.md)

Turtlebot
* [Turtlebot Physical Assembly](01-Turtlebot_Setup.md)

* [Load Turtlebot Device Drivers](03-Turtlebot_Bringup.md)

* [Teleoperate the Turtlebot](04-Turtlebot_Teleop.md)

* [Visualize Turtlebot Data](05-Turtlebot_Visualization.md)

SLAM
* [2D SLAM using GMapping](06-Gmapping.md)

* [3D SLAM with RGB-D Sensors](07-RGB-D_SLAM.md)
  * [SLAM Using RTABMAP](07b-RTABMAP.md)

Arduino
* [Use an Arduino with the Turtlebot](11-ROS_Arduino.md)
  * [Create a ROS Subscriber on the Arduino](11b-Arduino_Subscriber.md)
  * [Create a ROS Publisher on the Arduino](11c-Arduino_Publisher.md)

Fiducial Markers
* [View data from AprilTags](17-Fiducial_Markers.md)

PROGRAMMING
* [Intialize your workspace](08-Catkin_Workspace.md)
  * [Building ROSPY with your Catkin Package](08b-ROSPY_Building.md)
  * [Building ROSCPP with your Catkin Package](08c-ROSCPP_Building.md)

OpenCV
* [OpenCV and ROS](14-OpenCV2.md)
  * [OpenCV2 and ROSPY](14b-OpenCV2_Python.md)
  * [OpenCV2 and ROSCPP](14c-OpenCV2_CPP.md)

PCL
* [PCL and ROS](13-ROSPCL.md)

Quanergy M8
* [Quanergy M8](19-Quanergy_M8.md)

Data Collection
* [GUI Bagging](22b-rqt_turtlebot_dabit_bag.md)

## Resources

### Papers
SLAM
- [Online Global Loop Closure Detection for Large-Scale Multi-Session Graph-Based SLAM](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/e/eb/Labbe14-IROS.pdf)
- [Appearance-Based Loop Closure Detection for Online Large-Scale and Long-Term Operation](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/b/bc/TRO2013.pdf)
- [An Evaluation of 2D SLAM Techniques Available in Robot Operating System](http://home.isr.uc.pt/~davidbsp/publications/SPR_SSRR2013_SLAM.pdf)
- [SegMatch: Segment based loop-closure for 3D point clouds](https://arxiv.org/pdf/1609.07720.pdf)
- [A Loop Closure Improvement Method of Gmapping for Low Cost and Resolution Laser Scanner](https://www.sciencedirect.com/science/article/pii/S2405896316308278)
- [SLAM for Dummies](https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2005/projects/1aslam_blas_repo.pdf)

NAVIGATION
- [A Frontier-Based Approach for Autonomous Exploration](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated2/yamauchi_frontier_explor.pdf)

MOTION PLANNING
- [A Simple, but NP-Hard, Motion Planning Problem](http://msl.cs.uiuc.edu/%7Elericks4/papers/aaai13mcrnphard.pdf)
- 

CV/MV
- [Pedestrian Detection in RGB-D Data Using Deep Autoencoders](http://thescipub.com/PDF/ajassp.2015.847.856.pdf)
