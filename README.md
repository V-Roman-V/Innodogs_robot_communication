# Robot_Communication

Robot_Communications is a ROS package, which is a layer between the unitree legged SDK (on real robot) and the ROS.

The package provides an interface for sending the desired linear and angular velocity ([geometry_msgs::Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)) and receiving odometry data ([nav_msgs::Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)).

## Installing the package

First you need to build unitree legged sdk v3.3.1 from [unitreepy/unitree_legged_sdk/](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.3.1)

1. Download and place the sdk in the **robot_communication** package

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

This class uses leggedd sdk to interact with the robot:
It provides the functionality to send to the robot a target velocity and provides odometry from it.

## Parameters

The parameters file is in {robot_communication}/config/config.yaml

- **GlobalFrame**: Frame relative to which the robot's odometry will be published (default=*"map"*)

- **RobotFrame**: Robot Frame. Needed to display position and speed in the global frame system (default=*"robot"*)

- **OdometryTopic**: Odometry data will be published in this topic (default=*"robot/odometry"*)

- **VelocityTopic**: Target velocity will be subscribed to this topic (default=*"robot/target_velocity"*)

- **ControlHZ**: Frequency at which the robot will update the target velocity (default=*500*)

- **OdometryHZ**: Frequency with which the odometry will be published (default=*100*)


## How to launch

``` bash
roslaunch robot_communication main.launch
```