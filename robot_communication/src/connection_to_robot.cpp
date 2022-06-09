#include <robot_communication/connection_to_robot.hpp>

RobotConnection::RobotConnection(uint8_t level, const std::string& GlobalFrame, const std::string& RobotFrame)
    : safe(LeggedType::A1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)),
      GlobalFrame(GlobalFrame), RobotFrame(RobotFrame)
{
    udp.InitCmdData(cmd);
}

void RobotConnection::UDPRecv() {udp.Recv();}
void RobotConnection::UDPSend() {udp.Send();}
void RobotConnection::start_moving()   {enable=true;}
void RobotConnection::stop_moving()    {enable=false;}

nav_msgs::Odometry RobotConnection::getOdometry(ros::Time stamp){
  // ROS ODOMETRY:
  // This represents an estimate of a position and velocity in free space.  
  // The pose in this message should be specified in the coordinate frame given by header.frame_id.
  // The twist in this message should be specified in the coordinate frame given by the child_frame_id
  // -header         
  //   -seq           (uint32)      # sequence ID: consecutively increasing ID 
  //   -stamp         (time)        # time (nsec) since epoch
  //   -frame_id      (string)
  // -child_frame_id  (string)
  // -pose                          # This expresses position with uncertainty.
  //   -covariance    (float64[36]) # Row-major representation of the 6x6 covariance matrix
  //   -pose
  //     -position    (point{x,y,z})
  //     -orientation (quaternion{x,x,y,z})
  // -twist                         # This expresses velocity with uncertainty.
  //   -covariance    (float64[36]) # Row-major representation of the 6x6 covariance matrix
  //   -twist
  //     -linear      (Vector3{x,y,z}) 
  //     -angular     (Vector3{x,y,z})

  // ROBOT STATE:
  // state.imu;         // {quaternion[4]    - quaternion, normalized, (w,x,y,z);
                        //  gyroscope[3]     - angular velocity（rad/s)(raw data);
                        //  accelerometer[3] - m/(s^2)  (raw data);
                        //  rpy[3]           - euler angle (rad)
                        // }
  // state.position;    // (unit: m), from own odometry in inertial frame, usually drift
  // state.velocity;    // (unit: m/s), forwardSpeed, sideSpeed, ??rotateSpeed in body frame??

  static uint32 id = 0;

  nav_msgs::Odometry odom;
  odom.header.seq = id++;
  odom.header.stamp = stamp;
  odom.header.frame_id = GlobalFrame;

  odom.child_frame_id  = RobotFrame;

  odom.pose.pose.position.x = state.position[0];
  odom.pose.pose.position.y = state.position[1];
  odom.pose.pose.position.z = state.position[2];
  odom.pose.pose.orientation.w = state.imu.quaternion[0];
  odom.pose.pose.orientation.x = state.imu.quaternion[1];
  odom.pose.pose.orientation.y = state.imu.quaternion[2];
  odom.pose.pose.orientation.z = state.imu.quaternion[3];

  odom.twist.twist.linear.x = state.velocity[0];
  odom.twist.twist.linear.y = state.velocity[1];
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = state.imu.gyroscope[0];
  odom.twist.twist.angular.y = state.imu.gyroscope[1];
  odom.twist.twist.angular.z = state.imu.gyroscope[2];

  return odom;
}

  
void RobotConnection::setVelocity(const geometry_msgs::TwistConstPtr& twist){
  //update target velocity
  target_vel = *twist;
}

void RobotConnection::RobotControl() 
{
  // update state
  udp.GetRecv(state);

  // set default options
  cmd.mode = 0; //  0. idle, default stand; 2. target velocity walking (controlled by velocity + yawSpeed)
  cmd.gaitType = 0; // 0.idle  1.trot  2.trot running  3.climb stair
  cmd.speedLevel = 0; // 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
  cmd.footRaiseHeight = 0.08; // (unit: m, default: 0.08m), foot up height while walking
  cmd.bodyHeight = 0.28;      // (unit: m, default: 0.28m)
  cmd.euler[0]  = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;  // forwardSpeed (m/s)
  cmd.velocity[1] = 0.0f;  // sideSpeed in body frame (m/s)
  cmd.yawSpeed = 0.0f;     // (unit: rad/s), rotateSpeed in body frame

  if( enable ){
    // setting walking mode
    cmd.mode = 2;           
    cmd.gaitType = 0;       
    cmd.speedLevel = 0;     

    // setting velocity
    cmd.velocity[0] = target_vel.linear.x;
    cmd.velocity[1] = target_vel.linear.y;
    cmd.yawSpeed    = target_vel.angular.z; 
  }

  udp.SetSend(cmd);
}
