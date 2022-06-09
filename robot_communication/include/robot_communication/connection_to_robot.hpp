#pragma once

#include "ros/time.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

class RobotConnection
{
public:
  RobotConnection(uint8_t level, const std::string& GlobalFrame, const std::string& RobotFrame);
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  nav_msgs::Odometry getOdometry(ros::Time stamp);
  void setVelocity(const geometry_msgs::TwistConstPtr& twist);

  void stop_moving();
  void start_moving();

  float dt = 0.002;   // 0.001~0.01

private:
  bool enable = false;
  geometry_msgs::Twist target_vel;

  std::string GlobalFrame;
  std::string RobotFrame;

  Safety safe;
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
};