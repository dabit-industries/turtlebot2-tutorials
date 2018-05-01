## ROS Arduino

## Prequisites
This section **requires** the *catkin_ws* to be initialized and the *turtlebot_dabit* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)


This section **requires** the *rosserial_arduino* package to be setup.  
[Please click here to learn how to set up rosserial_arduino](11-ROS_Arduino.md)

## Sending data to the Arduino
As an example to send data to the Arduino, we will use the blink `example.o` tutorial. More informatino [can be pulled up, here](http://wiki.ros.org/rosserial_arduino/Tutorials/)

1. Upload the following code to your Arduino:

```c
/* rosserial Subscriber Example */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::Bool& msg){
  digitalWrite(13, msg.data);   // set the pin state to the message data
}

ros::Subscriber<std_msgs::Bool> sub("/arduino/led", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
```

## Connecting to the Arduino

1. Launch the arduino.launch
    * `source ~/catkin_ws/devel/setup.sh`
    * `roslaunch turtlebot_dabit arduino.launch`

## Communicating with the Arduino
1. Type
    * `rostopic list`
2. With the above example,and ROSSERIAL running, we can see in rostopic our Arduino topic:
    * __/arduino/led__
3. You can send data to this topic by using rostopic pub:
    * `rostopic pub /arduino/led std_msgs/Bool True`
    * `rostopic pub /arduino/led std_msgs/Bool False`

## Additional Tutorials
ROSSERIAL has recently improved their tutorials, and have a very wide selection of examples.
[Check out the ROSSerial_Arduino Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

## Troubleshooting
 

[Return to the main README page](/README.md)
