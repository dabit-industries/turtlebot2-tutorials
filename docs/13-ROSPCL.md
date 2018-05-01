## PCL with ROS using C++
Using PCL with ROS is possible using the PCL_ROS and ROS_PERCEPTION libraries.
This tutorial will show you how to get a message from a PointCloud2 topic in ROS, convert it to an pcl Point Cloud, and manipulate the point cloud.

## Prequisites
This example requires an image stream on the `/camera/rgb/image_raw` topic.
1. On the _turtlebot_, run 3dsensor.launch:
    1. `roslaunch turtlebot_bringup 3dsensor.launch`

This section **requires** the *catkin_ws* to be initialized and the *turtlebot_dabit* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)  

This section **requires** the *roscpp example* to be built in the *turtlebot_dabit* package.  
[Please click here to learn how to build turtlebot_dabit with roscpp](08c-ROSCPP_Building.md)  

## Getting a PointCloud from a ROS topic and using PCL
1. Copy your [example *roscpp_hello_world.cpp* code](/Setup/catkin_ws/src/turtlebot_dabit/src/roscpp_hello_world.cpp) to *roscpp_pcl.cpp*:
    1. `cp ~/catkin_ws/src/turtlebot_dabit/src/roscpp_hello_world.cpp ~/catkin_ws/src/turtlebot_dabit/src/roscpp_pcl.cpp`
    2. `gedit ~/catkin_ws/src/turtlebot_dabit/src/roscpp_pcl.cpp`

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
              pcl_conversions
              pcl_ros
            )
            ```

        * Replace `catkin_package(` with:

            ```
            catkin_package(
              INCLUDE_DIRS
              CATKIN_DEPENDS roscpp
                             pcl_conversions
                             pcl_ros
            )
            ```
        
        * Add your build target for *roscpp_opencv.cpp*:

            ```
            add_executable(roscpp_pcl_example src/roscpp_pcl_example.cpp)
            target_link_libraries(roscpp_pcl_example ${catkin_LIBRARIES})
            ```

    2. `gedit package.xml`
        * Add the build_depend and run_depends under roscpp:

            ```
            <build_depend>roscpp</build_depend>
            <build_depend>pcl_conversions</build_depend>
            <build_depend>pcl_ros</build_depend>
            <build_depend>libpcl-all-dev</build_depend>
            <run_depend>roscpp</run_depend>
            <run_depend>pcl_conversions</run_depend>
            <run_depend>pcl_ros</run_depend>
            <run_depend>libpcl-all</run_depend>
            ```

3. Edit *roscpp_pcl_example.cpp* in your *src* folder
    1. `gedit ~/catkin_ws/src/turtlebot_dabit/src/roscpp_pcl_example.cpp`
    2. Replace the *Hello ROS* code with the following *PCL* code:

        ```c
        /*
				 * PCL Example using ROS and CPP
				 */

				// Include the ROS library
				#include <ros/ros.h>

				// Include pcl
				#include <pcl_conversions/pcl_conversions.h>
				#include <pcl/point_cloud.h>
				#include <pcl/point_types.h>
				#include <pcl/filters/voxel_grid.h>

				// Include PointCloud2 message
				#include <sensor_msgs/PointCloud2.h>

				// Topics
				static const std::string IMAGE_TOPIC = "/camera/depth/points";
				static const std::string PUBLISH_TOPIC = "/pcl/points";

				// ROS Publisher
				ros::Publisher pub;

				void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
				{
					// Container for original & filtered data
					pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
					pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
					pcl::PCLPointCloud2 cloud_filtered;

					// Convert to PCL data type
					pcl_conversions::toPCL(*cloud_msg, *cloud);

					// Perform the actual filtering
					pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
					sor.setInputCloud (cloudPtr);
					sor.setLeafSize (0.1, 0.1, 0.1);
					sor.filter (cloud_filtered);

					// Convert to ROS data type
					sensor_msgs::PointCloud2 output;
					pcl_conversions::moveFromPCL(cloud_filtered, output);

					// Publish the data
					pub.publish (output);
				}

				int main (int argc, char** argv)
				{
					// Initialize the ROS Node "roscpp_pcl_example"
					ros::init (argc, argv, "roscpp_pcl_example");
					ros::NodeHandle nh;

					// Print "Hello" message with node name to the terminal and ROS log file
					ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

					// Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
					ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

					// Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
					pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

					// Spin
					ros::spin();

					// Success
					return 0;
				}
        ```
    
    3. Save and exit

4. Build and run your new code:
    1. `catkin_make --directory ~/catkin_ws --pkg turtlebot_dabit`
    2. `source ~/devel/setup.sh`
    3. `rosrun turtlebot_dabit roscpp_pcl_example`

## First Example Complete
* [~/catkin_ws/src/turtlebot_dabit/src/roscpp_pcl_example.cpp](/Setup/catkin_ws/src/turtlebot_dabit/src/roscpp_pcl_example.cpp)

    ```c
    /*
     * PCL Example using ROS and CPP
     */

    // Include the ROS library
    #include <ros/ros.h>

    // Include pcl
    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/filters/voxel_grid.h>

    // Include PointCloud2 message
    #include <sensor_msgs/PointCloud2.h>

    // Topics
    static const std::string IMAGE_TOPIC = "/camera/depth/points";
    static const std::string PUBLISH_TOPIC = "/pcl/points";

    // ROS Publisher
    ros::Publisher pub;

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2 cloud_filtered;

      // Convert to PCL data type
      pcl_conversions::toPCL(*cloud_msg, *cloud);

      // Perform the actual filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.1, 0.1, 0.1);
      sor.filter (cloud_filtered);

      // Convert to ROS data type
      sensor_msgs::PointCloud2 output;
      pcl_conversions::moveFromPCL(cloud_filtered, output);

      // Publish the data
      pub.publish (output);
    }

    int main (int argc, char** argv)
    {
      // Initialize the ROS Node "roscpp_pcl_example"
      ros::init (argc, argv, "roscpp_pcl_example");
      ros::NodeHandle nh;

      // Print "Hello" message with node name to the terminal and ROS log file
      ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

      // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
      ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

      // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
      pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

      // Spin
      ros::spin();

      // Success
      return 0;
    }
    ```

## Additional Resources
[ROSCPP Tutorials](http://wiki.ros.org/roscpp_tutorials)  
[ROSPCL Tutorials](http://wiki.ros.org/pcl/Tutorials)  

[Return to the main README page](/README.md)
