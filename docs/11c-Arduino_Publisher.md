## ROS Arduino

## Prequisites
This section **requires** the *catkin_ws* to be initialized and the *turtlebot_dabit* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)

This section **requires** the *rosserial_arduino* package to be setup.  
[Please click here to learn how to set up rosserial_arduino](11-ROS_Arduino.md)

## Receiving data from the Arduino
As an example to receive data from the Arduino, we will have the Arduino print the milliseconds since it has started.  
More information [can be pulled up, here](http://wiki.ros.org/rosserial_arduino/Tutorials/)

1. Upload the following code to your Arduino:

```c
/*
 * rosserial Publisher Example
 * Publishes the Arduino millis to the topic /arduino/millis
 */

/* Include the ROS library */
#include <ros.h>

/* Include the ROS message for an unsigned 32-bit integer
   Arduino's millis() command returns a `long`,
   which is an unsigned 32-bit integer */
#include <std_msgs/UInt32.h>

ros::NodeHandle  nh;

std_msgs::UInt32 millis_msg;
ros::Publisher pub_millis("/arduino/millis", &millis_msg);


void setup()
{
  // Initialize ROS Node
  nh.initNode();
  // Advertise the publisher topic
  nh.advertise(pub_millis);
}

void loop()
{
  // Set the millis_msg data to the millis() output
  millis_msg.data = millis();
  // Tell ROS to publish the data
  pub_millis.publish( &millis_msg );
  // Send the ROS message to the ROS Master
  nh.spinOnce();
  
  delay(5);
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
    * __/arduino/millis__
3. You can see the topic information using the rostopic command:
    * `rostopic info /arduino/millis`
3. You can see the data from the topic using the following command:
    * `rostopic echo /arduino/millis`

## Additional Tutorials
ROSSERIAL has recently improved their tutorials, and have a very wide selection of examples.
[Check out the ROSSerial_Arduino Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

## Troubleshooting
 

[Return to the main README page](/README.md)
