#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "robot_communication/connection_to_robot.hpp"

class Param{
private:
  void setStringParam(const std::string& name, std::string& param, const std::string& default){
    if (!nh.param<std::string>(name, param, default))
      ROS_WARN("%s parameter is set by default: %s",name, default);
    else
      ROS_INFO("%s parameter is set: %s", param.c_str());
  }
  void setIntParam(const std::string& name, int& param, const std::string& default){
    if (!nh.param(name, param, default))
      ROS_WARN("%s parameter is set by default: %s",name, default);
    else
      ROS_INFO("%s parameter is set: %s", param);
  }

public:
  Param(ros::NodeHandle& nh){
    setStringParam("GlobalFrame", GlobalFrame, "map");
    setStringParam("RobotFrame", RobotFrame, "robot");
    setStringParam("OdometryTopic", OdometryTopic, "robot/odometry");
    setStringParam("VelocityTopic", VelocityTopic, "robot/target_velocity");
    setIntParam("ControlHZ", ControlHZ, 100)
  }
    
  std::string GlobalFrame;
  std::string RobotFrame;

  std::string OdometryTopic;
  std::string VelocityTopic;

  int ControlHZ;
};

int main(int argc, char **argv)
{
  // INIT ROS
  ros::init(argc, argv, "robot_communication");
  ros::NodeHandle nh;
  Param param(nh);

  // INIT ROBODOG
  RobotConnection robot(HIGHLEVEL, param.GlobalFrame, param.RobotFrame);
  ROS_INFO("Communication start at HIGH-level.\n");

  // Create topics
  ros::Publisher  odom_pub  = nh.advertise<nav_msgs::Odometry>(param.OdometryTopic, 10);
  ros::Subscriber twist_sub = nh.subscribe(param.VelocityTopic, 10, &RobotConnection::setVelocity, &robot); // set callback to robot.setVelocity

  // Asynchronous communication with the robot
  LoopFunc loop_udpSend("udp_send", robot.dt, 3, boost::bind(&RobotConnection::UDPSend, &robot));
  LoopFunc loop_udpRecv("udp_recv", robot.dt, 3, boost::bind(&RobotConnection::UDPRecv, &robot));
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