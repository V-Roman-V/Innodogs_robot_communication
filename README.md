# Robot_Communication

Robot_Communications is a ROS package, which is a layer between the unitree legged SDK (on real robot) and the ROS itself.

The package provides an interface for sending the desired linear and angular velocity (geom:twist) and receiving odometry data.

## Installing the package

First you need to build unitree legged sdk v3.3.1 from [unitreepy/unitree_legged_sdk/](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.3.1)

1. Download and put sdk to root of the project

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
catkin build # or catkin_make
```

# Detail of Package

## unitree_legged_control:
It contains the joints controllers for Gazebo simulation, which allows users to control joints with position, velocity and torque. Refer to "unitree_ros/unitree_controller/src/servo.cpp" for joint control examples in different modes.

## The description of robots:
Namely the description of Go1, A1, Aliengo and Laikago. Each package include mesh, urdf and xacro files of robot. Take Laikago as an example, you can check the model in Rviz by:
```
roslaunch laikago_description laikago_rviz.launch
```

## unitree_gazebo & unitree_controller:
You can launch the Gazebo simulation by the following command:
```
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs
```
Where the `rname` means robot name, which can be `laikago`, `aliengo`, `a1` or `go1`. The `wname` means world name, which can be `earth`, `space` or `stairs`. And the default value of `rname` is `laikago`, while the default value of `wname` is `earth`. In Gazebo, the robot should be lying on the ground with joints not activated.

### Stand controller
After launching the gazebo simulation, you can start to control the robot:
```
rosrun unitree_controller unitree_servo
```

And you can add external disturbances, like a push or a kick:
```
rosrun unitree_controller unitree_external_force
```
### Position and pose publisher
Here we showed how to control the position and pose of robot without a controller, which should be useful in SLAM or visual development.

Then run the position and pose publisher in another terminal:
```
rosrun unitree_controller unitree_move_kinetic
```
The robot will turn around the origin, which is the movement under the world coordinate. And inside of the source file `move_publisher`, we also offered the method to move robot under robot coordinate. You can change the value of `def_frame` to `coord::ROBOT` and run the catkin_make again, then the `unitree_move_publisher` will move robot under its own coordinate.
