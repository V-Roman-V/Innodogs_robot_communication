#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cassert>
#include <thread>

#define assertm(exp, msg) assert(((void)msg, exp))

#include "robot_communication/connection_to_robot.hpp"

//Sets the parameters using ros
class Param{
  ros::NodeHandle *nh;

private:
  void setStringParam(const std::string& name, std::string& param, const std::string& dflt){
    if (!nh->param<std::string>(name, param, dflt))
      ROS_WARN("%s parameter is set by default: %s",name.c_str(), dflt.c_str());
    else
      ROS_INFO("%s parameter is set: %s",name.c_str(), param.c_str());
  }
  void setIntParam(const std::string& name, int& param, const int& dflt){
    if (!nh->param(name, param, dflt))
      ROS_WARN("%s parameter is set by default: %d",name.c_str(), dflt);
    else
      ROS_INFO("%s parameter is set: %d",name.c_str(), param);
  }

public:
  Param(ros::NodeHandle* nh)
    :nh(nh)
  {
    setStringParam("GlobalFrame", GlobalFrame, "map");
    setStringParam("RobotFrame", RobotFrame, "robot");
    setStringParam("OdometryTopic", OdometryTopic, "robot/odometry");
    setStringParam("VelocityTopic", VelocityTopic, "robot/target_velocity");
    setIntParam("ControlHZ", ControlHZ, 500);
    setIntParam("OdometryHZ", OdometryHZ, 100);

    assertm(ControlHZ != 0,"ControlHZ = 0, change the value in config file");
    assertm(OdometryHZ != 0,"OdometryHZ = 0, change the value in config file");
  }
    
  std::string GlobalFrame;
  std::string RobotFrame;

  std::string OdometryTopic;
  std::string VelocityTopic;

  int ControlHZ;
  int OdometryHZ;
};

// function for publishing odometry with some frequency
void publisherThread(int frequency, ros::Publisher* pub, RobotConnection* robot)
{
    ros::Rate rate(frequency);

    while (ros::ok()) {
      pub->publish( robot->getOdometry(ros::Time::now() ));
      rate.sleep();
    }
}

int main(int argc, char **argv)
{
  // INIT ROS
  ros::init(argc, argv, "robot_communication");
  ros::NodeHandle nh;
  Param param(&nh);

  // INIT ROBODOG
  RobotConnection robot(HIGHLEVEL, param.GlobalFrame, param.RobotFrame);
  ROS_INFO("Communication start at HIGH-level.\n");

  // Create topics
  ros::Publisher odom_pub  = nh.advertise<nav_msgs::Odometry>(param.OdometryTopic, 10);
  ros::Subscriber twist_sub = nh.subscribe(param.VelocityTopic, 10, &RobotConnection::setVelocity, &robot); // set callback to robot.setVelocity


  // Creating loop functions
  LoopFunc loop_udpSend("udp_send", 1./param.ControlHZ, 3, boost::bind(&RobotConnection::UDPSend, &robot));
  LoopFunc loop_udpRecv("udp_recv", 1./param.OdometryHZ, 3, boost::bind(&RobotConnection::UDPRecv, &robot));
  std::thread odom_thread(publisherThread, param.OdometryHZ, &odom_pub, &robot);
  loop_udpSend.start();
  loop_udpRecv.start();
  
  robot.start_moving();

  ros::spin();
  
  odom_thread.join();
  robot.stop_moving();
  return 0;
}