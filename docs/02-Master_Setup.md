## Laptop Setup
### Install Ubuntu 16.04 and ROS Kinetic using the Turtlebot 16.04 USB stick

1. Insert Turtlebot 16.04 USB stick into laptop
2. Power on laptop
2. Press `F12` to enter the [boot menu](https://support.lenovo.com/us/en/solutions/ht500222)
3. Select boot from flash drive device (`USB HDD: General UDisk`)
4. Follow the [Ubuntu Installation Guide](https://www.ubuntu.com/download/desktop/install-ubuntu-desktop)
    - Note: Check the box to allow the use of proprietary software.
5. Power off the laptop and remove the USB stick

### Troubleshoot: No Ubuntu in boot menu
1. Power on Laptop
2. Press `F12` to enter the [boot menu](https://support.lenovo.com/us/en/solutions/ht500222)
3. Select boot from flash drive device (`USB HDD: General UDisk`)
4. Boot into the live CD by selecting `Try Ubuntu`
5. Follow the [Boot-Repair](https://help.ubuntu.com/community/Boot-Repair) instructions. 

### Install ROS Kinetic Desktop-Full
1. [Follow the ROS Ubuntu installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Install Turtlebot packages
  ```bash
  sudo apt install ros-kinetic-turtlebot* ros-kinetic-astra-* -y
  ```
2. Install Other required packages:
  ```bash
  sudo apt install git chrony -y
  ```

3. (Optional) Install Turtlebot Branding:
```bash
mkdir ~/tmp && cd ~/tmp
git clone https://github.com/TurtleBot-Mfg/turtlebot-doc-indigo
git clone https://github.com/TurtleBot-Mfg/turtlebot-env-indigo
git clone https://github.com/TurtleBot-Mfg/turtlebot-branding-indigo
git clone https://github.com/TurtleBot-Mfg/turtlebot-wallpapers
sudo cp -r ~/tmp/turtlebot-branding-indigo/root/lib/plymouth/themes /usr/share/plymouth/themes
sudo cp -r ~/tmp/turtlebot-branding-indigo/root/usr/share/themes /usr/share/plymouth/themes
sudo cp -r ~/tmp/turtlebot-doc-indigo/root/etc/skel/* /etc/skel/.
cp ~/tmp/turtlebot-doc-indigo/root/etc/skel/Desktop/turtlebot-doc.desktop ~/Desktop
sudo cp -r ~/tmp/turtlebot-doc-indigo/root/usr/share/doc/turtlebot /usr/share/doc/.
sudo cp -r ~/tmp/turtlebot-env-indigo/root/etc/* /etc/.
sudo cp -r ~/tmp/turtlebot-env-indigo/root/usr/share/glib-2.0/schemas /usr/share/glib-2.0/schemas/.
sudo /usr/bin/glib-compile-schemas /usr/share/glib-2.0/schemas/
sudo cp -r ~/tmp/turtlebot-wallpapers/root/usr/share/backgrounds/* /usr/share/backgrounds/.
```

4. Install [Orbbec Astra](https://github.com/orbbec/ros_astra_camera) udev rules
```bash
mkdir ~/tmp
cd ~/tmp
wget https://raw.githubusercontent.com/orbbec/astra/master/install/orbbec-usb.rules
sudo cp orbbec-usb.rules /etc/udev/rules.d/.
```

5. Setup Turtlebot Parameters in Bashrc
```bash
echo export TURTLEBOT_BASE=kobuki >> ~/.bashrc
echo export TURTLEBOT_3D_SENSOR=astra >> ~/.bashrc
echo export TURTLEBOT_STACK=hexagons >> ~/.bashrc 
```

Due to incorrect NTP time servers, configure the same NTP zone between all ROS computers:
```bash
sudo ntpdate ntp.ubuntu.com
```
You may need to install `ntpdate` first:
```bash
sudo apt-get install ntpdate -y
```
 

[Return to the main README page](/README.md)
