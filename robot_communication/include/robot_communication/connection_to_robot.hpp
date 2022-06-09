#pragma once

#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

class RobotConnection
{
public:
  
  // Set the connection to the robot
  RobotConnection(uint8_t level, const std::string& GlobalFrame, const std::string& RobotFrame);   
  
  //Updates data received via udp. (Runs with some frequency)
  void UDPRecv();

  //Sends data from the send buffer. (Runs with some frequency) 
  void UDPSend(); 

  //Forms the odometry from the last received state
  nav_msgs::Odometry getOdometry(ros::Time stamp);

  //Sets the target velocity for the robot
  void setVelocity(const geometry_msgs::TwistConstPtr& twist); 

  //Stops control of the robot
  void stop_moving(); 

  //Enables robot control 
  void start_moving(); 

private:
  bool enable = false;

  std::string GlobalFrame;
  std::string RobotFrame;

  Safety safe;
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
};