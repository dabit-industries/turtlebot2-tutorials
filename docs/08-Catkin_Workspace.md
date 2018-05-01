## Catkin Workspace
[Catkin](http://wiki.ros.org/catkin) is a CMAKE-based build system used to build ROS packages.

## Catkin Setup
We will be setting up a Catkin Workspace on **BOTH** the *Turtlebot* and *Master* computers using the following steps.

1. Open a new terminal and create your catkin workspace
    1. Create a new directory in your home folder:
        * `mkdir -p ~/catkin_ws/src`
    2. CD to that directory:
        * `cd ~/catkin_ws/src`
    3. Initialize the workspace:
        * `catkin_init_workspace`
    4. Build the empty catkin workspace
        * `cd ~/catkin_ws`
        * `catkin_make`
    5. Source the setup.sh file to load the environment setup
        * `source ~/catkin_ws/devel/setup.sh`
2. Create a new catkin package called *turtlebot_dabit*
    1. CD to your catkin_ws src directory:
        * `cd ~/catkin_ws/src`
    2. Create a new catkin package:
        * `catkin_create_pkg turtlebot_dabit`
    3. CD into the new package:
        * `cd turtlebot_dabit`
    4. Make some directories for future use
        * `mkdir launch`
        * `mkdir scripts`
        * `mkdir src`

## What's Next?
[Building ROSPY with your Catkin Package](08b-ROSPY_Building.md)  
[Building ROSCPP with your Catkin Package](08c-ROSCPP_Building.md)  

## Catkin Tips
* You can create a new package and declare its dependencies at the same time:
    * `catkin_create_pkg turtlebot_dabit roscpp`
* You can build outside of the working directly by specifying `--directory /path/to/catkin/ws`:
    * `catkin_make --directory ~/catkin_ws`
* You can specify which packages to build using the `--pkg` argument:
    * `catkin_make --pkg turtlebot_dabit`



[Return to the main README page](/README.md)
