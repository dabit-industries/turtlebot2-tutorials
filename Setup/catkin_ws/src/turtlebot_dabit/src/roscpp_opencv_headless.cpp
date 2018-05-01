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

// Topics
static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
static const std::string PUBLISH_TOPIC = "/roscpp_opencv/output_video";

// Publisher
ros::Publisher pub;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  std_msgs::Header msg_header = msg->header;
  std::string frame_id = msg_header.frame_id.c_str();

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
  // TODO: std::asctime appends a '\n' character to the end of the string
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

  // Publish new image
  pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
  // Initialize the ROS Node "roscpp_opencv"
  ros::init(argc, argv, "roscpp_opencv");
  ros::NodeHandle nh;
  
  // Print "Hello" message with node name to the terminal and ROS log file
  ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

  // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function of image_cb
  ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, image_cb);

  // Create a ROS Publisher PUBLISH_TOPIC with a queue_size of 1
  pub = nh.advertise<sensor_msgs::Image>(PUBLISH_TOPIC, 1);

  // loop to keep ROS subscriber and publisher threads open
  ros::spin();

  // Program successful
  return 0;
}
