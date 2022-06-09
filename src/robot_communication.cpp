#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <robot_communication/connection_to_robot.hpp>

struct Param{
  Param(ros::NodeHandle& nh){
    if (!nh.param<std::string>("GlobalFrame", GlobalFrame, "map"))
      ROS_INFO("GlobalFrame parameter is set by default: %s\n", GlobalFrame.c_str());

    if (!nh.param<std::string>("RobotFrame", RobotFrame, "robot"))
      ROS_INFO("RobotFrame parameter is set by default: %s\n", RobotFrame.c_str());

    if (!nh.param<std::string>("OdometryTopic", OdometryTopic, "robot/odometry"))
      ROS_INFO("OdometryTopic parameter is set by default: %s\n", OdometryTopic.c_str());

    if (!nh.param<std::string>("VelocityTopic", VelocityTopic, "robot/target_velocity"))
      ROS_INFO("VelocityTopic parameter is set by default: %s\n", VelocityTopic.c_str());

    if (!nh.param("controlHZ", controlHZ, 100))
      ROS_INFO("controlHZ parameter is set by default: %d\n", controlHZ);
  }
    
  std::string GlobalFrame;
  std::string RobotFrame;

  std::string OdometryTopic;
  std::string VelocityTopic;

  int controlHZ;
};

int main(int argc, char **argv)
{
  // INIT ROS
  ros::init(argc, argv, "robot_communication");
  ros::NodeHandle nh;
  Param param(nh);
  ros::Publisher  odom_pub  = nh.advertise<nav_msgs::Odometry>(param.OdometryTopic, 10);
  ros::Subscriber twist_sub = nh.subscribe(param.VelocityTopic, 10, robot.setVelocity); // set callback to robot.setVelocity

  // INIT ROBODOG
  RobotConnection robot(HIGHLEVEL);
  ROS_INFO("Communication start at HIGH-level.\n");

  // asynchronous communication with the robot
  LoopFunc loop_udpSend("udp_send", robot.dt, 3, boost::bind(&RobotInteraction::UDPSend, &robot));
  LoopFunc loop_udpRecv("udp_recv", robot.dt, 3, boost::bind(&RobotInteraction::UDPRecv, &robot));
  loop_udpSend.start();
  loop_udpRecv.start();
  robot.start_moving();

  ros::Rate loop_rate(param.controlHZ);
  while (ros::ok())
  {
    robot.RobotControl();
    odom_pub.publish( robot.getOdometry(ros::Time::now()) );
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  robot.stop_moving();
  return 0;
}