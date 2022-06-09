# Robot_Communication

Robot_Communications is a ROS package, which is a layer between the unitree legged SDK (on real robot) and the ROS.

The package provides an interface for sending the desired linear and angular velocity ([geometry_msgs::Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)) and receiving odometry data ([nav_msgs::Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)).

## Installing the package

First you need to build unitree legged sdk v3.3.1 from [unitreepy/unitree_legged_sdk/](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.3.1)

1. Download and place the sdk in the root of the project

2. Build the sdk

    ```bash
    cd unitree_legged_sdk
    mkdir build
    cd build
    cmake ../
    make
    ```

To do so the following dependencies should be met:

- [Boost](http://www.boost.org) (version 1.5.4 or higher)
- [CMake](http://www.cmake.org) (version 2.8.3 or higher)
- [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)

Then you can use catkin_make to build:

``` bash
cd ~/catkin_ws
catkin_make # or catkin build
```

## Detail of Package

### Class ConnectionToRobot

This class uses the leggett sdk to interact with the robot and sends the target speed to the robot via the interface with some frequency (). This class also provides information about the last odometry received from the robot.