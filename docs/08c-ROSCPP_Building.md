## Setting up the catkin package to build ROSCPP
In order to compile and run our C++ code, we need to make sure our ROS package has those build capabilities.  

## Prequisites
This section **requires** the *catkin_ws* to be initialized and the *turtlebot_dabit* package created.  
[Please click here to learn how to initialize the catkin workspace](08-Catkin_Workspace.md)


## Catkin Package Setup
For this tutorial, we will be using the *turtlebot_dabit* catkin package we created.
In order to enable building C++ files in our package, we need to edit the *CMakeLists.txt* and *package.xml* to build with our dependencies.
1. Source your catkin workspace and cd to the *turtlebot_dabit* package:
    1. `cd ~/catkin_ws/src/turtlebot_dabit`
2. Edit your *package.xml* and add build and run dependencies:
    1. `gedit package.xml`
        * Near the bottom of the file, find the `<buildtool_depend>catkin</buildtool_depend>` line:

            ```xml
            ...
            <buildtool_depend>catkin</buildtool_depend>
            ...
            ```

        * Add the build_depend and run_depend tags for roscpp **underneath** the *buildtool_depend* line:

            ```xml
            <build_depend>roscpp</build_depend>
            <run_depend>roscpp</run_depend>
            ```

        * [The resulting package.xml](/Setup/catkin_ws/src/turtlebot_dabit/package.xml)

            ```xml
            <?xml version="1.0"?>
            <package>
              <name>turtlebot_dabit</name>
              <version>0.0.0</version>
              <description>The turtlebot_dabit package</description>

              <!-- One maintainer tag required, multiple allowed, one person per tag -->
              <!-- Example:  -->
              <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
              <maintainer email="user@todo.todo">user</maintainer>


              <!-- One license tag required, multiple allowed, one license per tag -->
              <!-- Commonly used license strings: -->
              <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
              <license>TODO</license>

              <!-- ... -->

              <buildtool_depend>catkin</buildtool_depend>
              <build_depend>roscpp</build_depend>
              <run_depend>roscpp</run_depend>

              <!-- The export tag contains other, unspecified, tags -->
              <export>
                <!-- Other tools can request additional information be placed here -->

              </export>
            </package>
            ```

3. Edit your *CMakeLists.txt* and add your dependencies to it
    1. `gedit CMakeLists.txt`
        * Near the top of the file, find the `find_package(catkin REQUIRED)` line:

            ```
            ...
            find_package(catkin REQUIRED)
            ...
            ```

        * Replace that line with:

            ```
            find_package(catkin REQUIRED COMPONENTS
              roscpp
            )
            ```
 
        * Near the middle of the file, find the `catkin_package(` line:
 
            ```
            ...
            catkin_package(
            #  INCLUDE_DIRS include
            #  LIBRARIES turtlebot_dabit
            #  CATKIN_DEPENDS other_catkin_pkg
            #  DEPENDS system_lib
            )
            ...
            ```
 
        * Replace that line with:
 
            ```
            catkin_package(
              INCLUDE_DIRS
              CATKIN_DEPENDS roscpp
            )
            ```
 
        * Near the middle of the file, find the `# include_directories(include)` line:
 
            ```
            # include_directories(include)
            ```
 
        * Replace that line with:
 
            ```
            include_directories(
              ${catkin_INCLUDE_DIRS}
            )
            ```
 
        * [The resulting CMakeLists.txt](/Setup/catkin_ws/src/turtlebot_dabit/CMakeLists.txt)
 
            ```
            cmake_minimum_required(VERSION 2.8.3)
            project(turtlebot_dabit)
       
            ## Add support for C++11, supported in ROS Kinetic and newer
            # add_definitions(-std=c++11)
       
            ## Find catkin macros and libraries
            ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
            ## is used, also find other catkin packages
            find_package(catkin REQUIRED COMPONENTS
              roscpp
            )
       
            #  ...
       
            ###################################
            ## catkin specific configuration ##
            ###################################
            ## The catkin_package macro generates cmake config files for your package
            ## Declare things to be passed to dependent projects
            ## INCLUDE_DIRS: uncomment this if you package contains header files
            ## LIBRARIES: libraries you create in this project that dependent projects also need
            ## CATKIN_DEPENDS: catkin_packages dependent projects also need
            ## DEPENDS: system dependencies of this project that dependent projects also need
            catkin_package(
              INCLUDE_DIRS
              CATKIN_DEPENDS roscpp
            )
       
            ###########
            ## Build ##
            ###########
            ## Specify additional locations of header files
            ## Your package locations should be listed before other locations
            include_directories(
              ${catkin_INCLUDE_DIRS}
            )
       
       
            ## ...
            ```
 
4. Test your new package configuration is correct by running catkin_make:
    * `catkin_make --directory ~/catkin_ws --pkg turtlebot_dabit`
    * Upon success, you should see:
 
        ```
        -- Configuring done
        -- Generating done
        -- Build files have been written to: /home/user/catkin_ws/build
        ####
        #### Running command: "make -j4 -l4" in "/home/user/catkin_ws/build/turtlebot_dabit"
        ####
        ```
 
## Writing the first ROSCPP C++ code
Now we get to write our first c++ code to communicate with ROS.  
1. Create a new file titled [*roscpp_hello_world.cpp*](/Setup/catkin_ws/src/turtlebot_dabit/src/roscpp_hello_world.cpp) in your *src* folder
    1. `gedit ~/catkin_ws/src/turtlebot_dabit/src/roscpp_hello_world.cpp`
    2. Start with the following *Hello World* code in gedit:
 
        ```c
        /*
         * Hello World Example using ROS and CPP
         */
 
        // Include the ROS library
        #include <ros/ros.h>
 
        // Main function
        int main(int argc, char** argv)
        { 
          // Initialize the ROS Node "roscpp_hello_world"
          ros::init(argc, argv, "roscpp_hello_world");
 
          // Instantiate the ROS Node Handler as nh
          ros::NodeHandle nh;
          
          // Print "Hello ROS!" to the terminal and ROS log file
          ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());
          
          // Program succesful
          return 0;
        } 
        ```
 
    3. Save and exit *roscpp_hello_world.cpp*
2. Edit your *CMakeLists.txt* and specify to build *roscpp_hello_world.cpp*:
    1. `gedit ~/catkin_ws/src/turtlebot_dabit/CMakeLists.txt`
        * Near the middle of the file, find the `## Declare a C++ executable`:
 
            ```
            ...
            ## Declare a C++ executable
            ## With catkin_make all packages are built within a single CMake context
            ## The recommended prefix ensures that target names across packages don't collide
            # add_executable(${PROJECT_NAME}_node src/turtlebot_dabit_node.cpp)
            ...
            ```

        * Below that line, add the following:

            ```
            add_executable(roscpp_hello_world src/roscpp_hello_world.cpp)
            target_link_libraries(roscpp_hello_world ${catkin_LIBRARIES})
            ```

    2. Save *CMakeLists.txt* and exit gedit
3. Compile your code using catkin_make:
    1. `catkin_make --directory ~/catkin_ws --pkg turtlebot_dabit`
        * Upon success, you should see:

            ```
            ####
            #### Running command: "make -j4 -l4" in "/home/user/catkin_ws/build/turtlebot_dabit"
            ####
            Scanning dependencies of target roscpp_hello_world
            [ 50%] Building CXX object turtlebot_dabit/CMakeFiles/roscpp_hello_world.dir/src/roscpp_hello_world.cpp.o
            [100%] Linking CXX executable /home/user/catkin_ws/devel/lib/turtlebot_dabit/roscpp_hello_world
            [100%] Built target roscpp_hello_world
            ```

4. Run your code using rosrun:
    1. `source ~/catkin_ws/devel/setup.sh`
    2. `rosrun turtlebot_dabit roscpp_hello_world`
        * Upon success, you should see: `[ INFO] [1492726164.127098818]: Hello ROS!`


## Additional Resources
[ROSCPP Tutorials](http://wiki.ros.org/roscpp_tutorials)  



[Return to the main README page](/README.md)

