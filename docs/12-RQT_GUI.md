## RQT_GUI

1. Create Catkin (Build) Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
2. Clone tutorial repo into workspace
```bash
git clone https://github.com/dabit-industries/turtlebot2-tutorials
```
3. Go to directory containing GUI code:
```bash
cd ~/catkin_ws/src/rqt_turtlebot_dabit
```

4. Install dependencies
```bash
sudo apt install python-qt-binding python3-pyqt5 -y
```
5. Build the Workspace
```bash
cd ~/catkin_ws/src/turtlebot2-tutorials/catkin_ws
catkin_make
```
Clean rqt cache
```
rm ~/.config/ros.org/rqt_gui.ini
```
6. Source the bash files
```bash
 source ~/.bashrc
 source ~/catkin_ws/devel/setup.bash
 ```
7. Run the interface
```bash
 rosrun rqt_turtlebot_dabit rqt_turtlebot_dabit
```

![](Resources/00-turtlebot_dabit_gui.gif)
 

[Return to the main README page](/README.md)
