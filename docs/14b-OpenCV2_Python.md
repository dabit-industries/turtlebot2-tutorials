## OpenCV with ROS using Python
Using OpenCV with ROS is possible using the CvBridge library.  
This tutorial will show you how to get a message from an Image topic in ROS, convert it to an OpenCV Image, and manipulate the image.  

## Prequisites
This example requires an image stream on the `/camera/rgb/image_raw` topic.  
1. On the _turtlebot_, run 3dsensor.launch:
    1. `roslaunch turtlebot_bringup 3dsensor.launch`
    
This section **requires** the *catkin_ws* to be initialized and the *turtlebot_dabit* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)

## Getting an Image from a ROS Topic using Python
The following example code can be used on __either__ the _master_ or _turtlebot_ computers.  
1. Create a new Python file in your scripts folder in your workspace:
    1. `mkdir -p ~/catkin_ws/src/turtlebot_dabit/scripts`
    2. `gedit ~/catkin_ws/src/turtlebot_dabit/scripts/rospy_example.py`
2. Start the script by specifying which Python version to use:

    ```python
    #!/usr/bin/env python2.7

    # Print "Hello!" to terminal
    print "Hello!"
    ```

    * Test the script in your terminal:
        * `python ~/catkin_ws/src/turtlebot_dabit/scripts/opencv_example_1.py`
3. Import the neccesary ROS and Computer Vision libraries and Initialize the ROS Node

    ```python
    #!/usr/bin/env python2.7
    # Import ROS libraries and messages
    import rospy

    # Print "Hello!" to terminal
    print "Hello!"

    # Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
    rospy.init_node('opencv_example', anonymous=True)

    # Print "Hello ROS!" to the Terminal and ROSLOG
    rospy.loginfo("Hello ROS!")
    ```

    * Test the script in your terminal:
        * `python ~/catkin_ws/src/turtlebot_dabit/scripts/opencv_example_1.py`
5. Create a subscriber for an Image topic, and define a callback function  
   Use _CTRL+C_ to stop the program

    ```python
    ...

    import rospy
    from sensor_msgs.msg import Image

    ...

    # Define a callback for the Image message
    def image_callback(img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()
    ```

    * Test the script in your terminal:
        * `python ~/catkin_ws/src/turtlebot_dabit/scripts/opencv_example_1.py`
6. Import OpenCV and cv_bridge, create a window to show a live image in  
   Use _CTRL+C_ in the terminal to stop the program

    ```python
    ...
    # Import ROS libraries and messages
    ...

    # Import OpenCV libraries and tools
    import cv2
    from cv_bridge import CvBridge, CvBridgeError

    ...

    # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
    rospy.loginfo("Hello ROS!")

    # Initialize the CvBridge class
    bridge = CvBridge()

    # Define a function to show the image in an OpenCV Window
    def show_image(img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)

    # Define a callback for the Image message
    def image_callback(img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)

        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Show the converted image
        show_image(cv_image)

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Initialize an OpenCV Window named "Image Window"
    cv2.namedWindow("Image Window", 1)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()
    ```

    * Test the script in your terminal:
        * `python ~/catkin_ws/src/turtlebot_dabit/scripts/opencv_example_1.py`
7. Rotate the image 90 degrees  
   Use _CTRL+C_ in the terminal to stop the program

    ```python
    ...
    # Define a callback for the Image message
    define image_callback(img_msg):
        ...

        # Flip the image 90deg
        cv_image = cv2.transpose(cv_image)
        cv_image = cv2.flip(cv_image,1)

        # Show the converted image
        show_image(cv_image)

    ...
    ```

    * Test the script in your terminal:
        * `python ~/catkin_ws/src/turtlebot_dabit/scripts/opencv_example_1.py`
8. Allow execution permissions for your code, and use rosrun to run it!  
   In a new terminal:
    1. `chmod +x ~/catkin_ws/src/turtlebot_dabit/scripts/opencv_example_1.py`
    2. `source ~/catkin_ws/devel/setup.sh`
    3. `rosrun turtlebot_dabit opencv_example_1.py`

## First Example Complete
* [`~/catkin_ws/src/turtlebot_dabit/scripts/rospy_opencv.py`](/Setup/catkin_ws/src/turtlebot_dabit/scripts/rospy_opencv.py)

    ```python
    #!/usr/bin/env python2.7
    # Import ROS libraries and messages
    import rospy
    from sensor_msgs.msg import Image

    # Import OpenCV libraries and tools
    import cv2
    from cv_bridge import CvBridge, CvBridgeError

    # Print "Hello!" to terminal
    print "Hello!"

    # Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
    rospy.init_node('opencv_example', anonymous=True)

    # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
    rospy.loginfo("Hello ROS!")

    # Initialize the CvBridge class
    bridge = CvBridge()

    # Define a function to show the image in an OpenCV Window
    def show_image(img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)

    # Define a callback for the Image message
    def image_callback(img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)

        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Flip the image 90deg
        cv_image = cv2.transpose(cv_image)
        cv_image = cv2.flip(cv_image,1)

        # Show the converted image
        show_image(cv_image)

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Initialize an OpenCV Window named "Image Window"
    cv2.namedWindow("Image Window", 1)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()
    ```

## Additional Resources
[ROSPY Tutorials](http://wiki.ros.org/rospy_tutorials/Tutorials)  
[CV_Bridge Tutorials](http://wiki.ros.org/cv_bridge/Tutorials)  


[Return to the main README page](/README.md)
