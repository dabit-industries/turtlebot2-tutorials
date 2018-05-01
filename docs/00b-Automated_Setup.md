## Automated Environment Setup
This tool sets up the `turtlebot2-tutorials` environment so that the tutorials can run without setup. It assumes that the [ROS setup](02-Master_Setup.md) has been completed.  

## Installation
Use [git](http://rogerdudler.github.io/git-guide/
) to copy this entire repository onto your *Turtlebot* and *Master* computer and follow the other commands:
```bash
git clone --recursive https://github.com/dabit-industries/turtlebot2-tutorials ~/turtlebot2-tutorials
```

If you'd like the most recent version of the code on this site, you can update it with following commands:
```
cd ~/turtlebot2-tutorials
git pull
```

## Setup
These commands are to be run on both the **Master** and **Turtlebot** computer
1. On the command line, navigate to the repository folder:
    1. `cd ~/turtlebot2-tutorials`
2. Run the environment setup program:
    1. `source ~/turtlebot2-tutorials/Setup/automated/automate.sh`
        * You should see: `Environment setup successful`
3. Run our utility to check, copy, and compile all the code:
    1. `dabit-setup-utility install`
        * This code requires some interaction, please follow along in the terminal
        * This may take a while as it is getting all the dependencies and building all the code
        * This code moves the existing `~/catkin_ws` and `~/workspace` to `~/backup/old_workspace_DD-MM-YY_hh-mm-ss`
    2. On the **Master** Computer, set your *MASTER_IP*:
        * `change_master IP_OF_TURTLEBOT`
4. Test a code example:
    1. In a new terminal:
        * type: `roslaunch turtlebot_bringup minimal.launch`
    2. In a new terminal:
        * type: `rosrun turtlebot_dabit roscpp_hello_world`
        * You should see the following in your terminal:
        
            ```
            [ INFO] [1492930878.699859775]: Hello from ROS node /roscpp_example
            ```

## Running Updates
These commands are to be run on both the **master** and **turtlebot** computer
1. On the command line, use our setup utility to do an update:
    * `dabit-setup-utility update`
2. Reset your workspace:
    * `dabit-setup-utility reset`
        * This code moves the existing `~/catkin_ws` and `~/workspace` to `~/backup/old_workspace_DD-MM-YY_hh-mm-ss`
3. Install the new workspace:
    * `dabit-setup-utility install`
        * This code requires some interaction, please follow along in the terminal
        * This may take a while as it is getting all the dependencies and building all the code
        * This code moves the existing `~/catkin_ws` and `~/workspace` to `~/backup/old_workspace_DD-MM-YY_hh-mm-ss`
4. On the Turtlebot, open a terminal and type:
    * `roslaunch turtlebot_dabit turtlebot.launch`

## Running ROSPY Sample Code:
* `rosrun turtlebot_dabit rospy_hello_world.py`
* `rosrun turtlebot_dabit rospy_opencv.py`
    * You need to run `roslaunch turtlebot_bringup 3dsensor.launch` before running this code
* `rosrun turtlebot_dabit rospy_example_class`

## Running ROSCPP Sample Code:
* `rosrun turtlebot_dabit roscpp_hello_world`
* `rosrun turtlebot_dabit roscpp_opencv`
    * You need to run `roslaunch turtlebot_bringup 3dsensor.launch` before running this code
* `rosrun turtlebot_dabit roscpp_pcl_example`
* `rosrun turtlebot_dabit roscpp_publisher`
* `rosrun turtlebot_dabit roscpp_subscriber`

## Running ROS Arduino:
1. `roslaunch turtlebot_dabit arduino.launch`

## Running Wanderer Code:
**WORK-IN-PROGRESS**

## TROUBLESHOOTING:
1. *rqt* applications don't run
    * I.E. `rqt_image_view` doesn't run
    * `sudo apt remove python-qt4`
