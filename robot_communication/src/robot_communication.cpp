#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

#include "robot_communication/connection_to_robot.hpp"

//Sets the parameters using ros
class Param{
  ros::NodeHandle nh;

private:
  void setStringParam(const std::string& name, std::string& param, const std::string& dflt){
    if (!nh.param<std::string>(name, param, dflt))
      ROS_WARN("%s parameter is set by default: %s",name.c_str(), dflt.c_str());
    else
      ROS_INFO("%s parameter is set: %s",name.c_str(), param.c_str());
  }
  void setIntParam(const std::string& name, int& param, const int& dflt){
    if (!nh.param(name, param, dflt))
      ROS_WARN("%s parameter is set by default: %d",name.c_str(), dflt);
    else
      ROS_INFO("%s parameter is set: %d",name.c_str(), param);
  }

public:
  Param(ros::NodeHandle& nh){
    setStringParam("GlobalFrame", GlobalFrame, "map");
    setStringParam("RobotFrame", RobotFrame, "robot");
    setStringParam("OdometryTopic", OdometryTopic, "robot/odometry");
    setStringParam("VelocityTopic", VelocityTopic, "robot/target_velocity");
    setIntParam("ControlHZ", ControlHZ, 500);
    setIntParam("OdometryHZ", OdometryHZ, 100);

    if(ControlHZ == 0)
      ROS_WARN("ControlHZ = 0, robot control is disabled");
    if(OdometryHZ == 0)
      ROS_WARN("OdometryHZ = 0, odometry publishing is disabled");
    
    this->nh = nh;
  }
    
  std::string GlobalFrame;
  std::string RobotFrame;

  std::string OdometryTopic;
  std::string VelocityTopic;

  int ControlHZ;
  int OdometryHZ;
};

// class for publshing odometry from robot
class Publisher{
public:
  Publisher(ros::NodeHandle& nh, const std::string& topic, RobotConnection* robot)
    :robot(robot)
  {
    ros::Publisher odom_pub  = nh.advertise<nav_msgs::Odometry>(topic, 10);
  }
  void publish(){
    odom_pub.publish( robot->getOdometry(ros::Time::now() ));
  }

private:
  RobotConnection* robot;
  ros::Publisher odom_pub;
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
  Publisher odom_pub(nh, param.OdometryTopic, &robot);
  ros::Subscriber twist_sub = nh.subscribe(param.VelocityTopic, 10, &RobotConnection::setVelocity, &robot); // set callback to robot.setVelocity


  // Creating loop functions
  LoopFunc loop_udpSend("udp_send", 1./param.ControlHZ, 3, boost::bind(&RobotConnection::UDPSend, &robot));
  LoopFunc loop_udpRecv("udp_recv", 1./param.OdometryHZ, 3, boost::bind(&RobotConnection::UDPRecv, &robot));
  LoopFunc loop_pubOdom("pub_odom", 1./param.OdometryHZ,   boost::bind(&Publisher::publish,       &odom_pub));
  
  if(param.ControlHZ != 0){
    loop_udpSend.start();
  }
  if(param.OdometryHZ != 0){
    loop_udpRecv.start();
    loop_pubOdom.start();
  }
  
  robot.start_moving();

  ros::spin();

  robot.stop_moving();
  return 0;
}