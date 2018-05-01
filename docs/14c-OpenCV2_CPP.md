## OpenCV with ROS using C++
Using OpenCV with ROS is possible using the CvBridge library.
This tutorial will show you how to get a message from an Image topic in ROS, convert it to an OpenCV Image, and manipulate the image.

## Prequisites
This example requires an image stream on the `/camera/rgb/image_raw` topic.
1. On the _turtlebot_, run 3dsensor.launch:
    1. `roslaunch turtlebot_bringup 3dsensor.launch`

This section **requires** the *catkin_ws* to be initialized and the *turtlebot_dabit* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)  

This section **requires** the *roscpp example* to be built in the *turtlebot_dabit* package.  
[Please click here to learn how to build turtlebot_dabit with roscpp](08c-ROSCPP_Building.md)  


## Getting an Image from a ROS topic and using OpenCV
1. Copy your [example *roscpp_hello_world.cpp* code](/Setup/catkin_ws/src/turtlebot_dabit/src/roscpp_hello_world.cpp) to *roscpp_opencv.cpp*:
    1. `cp ~/catkin_ws/src/turtlebot_dabit/src/roscpp_hello_world.cpp ~/catkin_ws/src/turtlebot_dabit/src/roscpp_opencv.cpp`
    2. `gedit ~/catkin_ws/src/turtlebot_dabit/src/roscpp_opencv.cpp`

        ```c
        /*
         * Hello World Example using ROS and CPP
         */

        // Include the ROS library
        #include <ros/ros.h>

        // Main function
        int main(int argc, char** argv)
        {
          // Initialize the ROS Node "roscpp_example"
          ros::init(argc, argv, "roscpp_example");

          // Instantiate the ROS Node Handler as nh
          ros::NodeHandle nh;

          // Print "Hello ROS!" to the terminal and ROS log file
          ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());

          // Program succesful
          return 0;
        }
        ```

2. Set up your package dependencies:
    1. `gedit ~/CMakeLists.txt`
        * Replace `find_package(catkin REQUIRED COMPONENTS)` with:

            ```
            find_package(catkin REQUIRED COMPONENTS
              roscpp
              sensor_msgs
              std_msgs
              cv_bridge
              image_transport
            )

            find_package(OpenCV 2 REQUIRED)
            ```

        * Replace `catkin_package(` with:

            ```
            catkin_package(
              INCLUDE_DIRS
              CATKIN_DEPENDS roscpp
                             sensor_msgs
                             std_msgs
                             cv_bridge
                             image_transport
            )
            ```

        * Replace your `include_directories(` with:
        
            ```
            include_directories(
              ${catkin_INCLUDE_DIRS}
              ${OpenCV_INCLUDE_DIRS}
            )
            ```

        * Add your build target for *roscpp_opencv.cpp*:

            ```
            add_executable(roscpp_opencv_example src/roscpp_opencv_example.cpp)
            target_link_libraries(roscpp_opencv_example ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
            ```

3. Edit *opencv_example.cpp* in your *src* folder
    1. `gedit ~/catkin_ws/src/turtlebot_dabit/src/opencv_example.cpp`
    2. Replace the *Hello ROS* code with the following *OpenCV* code:

        ```c
        /*
         * OpenCV Example using ROS and CPP
         */

        // Include the ROS library
        #include <ros/ros.h>

        // Include opencv2
        #include <opencv2/imgproc/imgproc.hpp>
        #include <opencv2/highgui/highgui.hpp>

        // Include CvBridge, Image Transport, Image msg
        #include <image_transport/image_transport.h>
        #include <cv_bridge/cv_bridge.h>
        #include <sensor_msgs/image_encodings.h>

        // OpenCV Window Name
        static const std::string OPENCV_WINDOW = "Image window";

        // Topics
        static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
        static const std::string PUBLISH_TOPIC = "/image_converter/output_video";

        // Publisher
        ros::Publisher pub;

        void image_cb(const sensor_msgs::ImageConstPtr& msg)
        {
          std_msgs::Header msg_header = msg->header;
          std::string frame_id = msg_header.frame_id.c_str();
          ROS_INFO_STREAM("New Image from " << frame_id);

          cv_bridge::CvImagePtr cv_ptr;
          try
          {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          }
          catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
          }

          // Draw an example crosshair
          cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

          // Update GUI Window
          cv::imshow(OPENCV_WINDOW, cv_ptr->image);
          cv::waitKey(3);

          // Output modified video stream
          pub.publish(cv_ptr->toImageMsg());
        }
        ```
    
    3. Save and exit

4. Build and run your new code:
    1. `catkin_make --directory ~/catkin_ws --pkg turtlebot_dabit`
    2. `source ~/devel/setup.sh`
    3. `rosrun turtlebot_dabit opencv_example`


## First Example Complete
* [~/catkin_ws/src/turtlebot_dabit/src/opencv_example.cpp](/Setup/catkin_ws/src/turtlebot_dabit/src/roscpp_opencv.cpp)

    ```c
    /*
     * OpenCV Example using ROS and CPP
     */

    // Include the ROS library
    #include <ros/ros.h>

    // Include opencv2
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/highgui/highgui.hpp>

    // Include CvBridge, Image Transport, Image msg
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/image_encodings.h>

    // OpenCV Window Name
    static const std::string OPENCV_WINDOW = "Image window";

    // Topics
    static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
    static const std::string PUBLISH_TOPIC = "/image_converter/output_video";

    // Publisher
    ros::Publisher pub;

    void image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
      std_msgs::Header msg_header = msg->header;
      std::string frame_id = msg_header.frame_id.c_str();
      ROS_INFO_STREAM("New Image from " << frame_id);

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Draw a timestamp of the current date and time in the top left of the image
      // FIX-ME: std::asctime appends a '\n' character to the end of the string
      std::time_t result = msg_header.stamp.sec;
      std::stringstream ss;
      ss << std::asctime(std::localtime(&result));

      // Get the size of the text for measurement
      cv::Size text = cv::getTextSize(ss.str().c_str(), CV_FONT_HERSHEY_SIMPLEX, 0.4, 1, 0);

      // Put the text in the bottom right corner
      cv::Point text_point = cvPoint(cv_ptr->image.cols - 20 - text.width, cv_ptr->image.rows - 20 - text.height);

      // Draw a black background behind text
      cv::rectangle(cv_ptr->image, text_point, text_point + cv::Point(text.width, -text.height), CV_RGB(0,0,0), CV_FILLED);

      // Draw the timestamp on the rectangle
      cv::putText(cv_ptr->image, ss.str().c_str(), text_point, CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255));

      // Draw an example circle on the video stream
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      // Draw an example crosshair
      cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      // Output modified video stream
      pub.publish(cv_ptr->toImageMsg());
    }

    ```

## Additional Resources
[ROSCPP Tutorials](http://wiki.ros.org/roscpp_tutorials)  
[Cv_Bridge Tutorials](http://wiki.ros.org/cv_bridge/Tutorials)  



[Return to the main README page](/README.md)
