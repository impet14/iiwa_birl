#ifndef FORCETRACKSTIFFNESS_H
#define FORCETRACKSTIFFNESS_H
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen3/Eigen/Eigen>

#include "iiwa_msgs/ConfigureSmartServo.h"
#include "iiwa_msgs/ControlMode.h"

class ForceTrackStiffness{
private:
  ros::NodeHandle nh;
  ///subscrib the Cartesian wrench
  ros::Subscriber wrench_sub;
  ///The client can change the control mode
  ros::ServiceClient client;
  ///publish the force error.
  ros::Publisher force_error_pub;
  ///the desired force
  Eigen::Matrix<float, 6, 1> desired_force;

  iiwa_msgs::ConfigureSmartServo config;
  ///the proportion
  const float k_p;
public:
  ForceTrackStiffness(const ros::NodeHandle &nh_);
  ~ForceTrackStiffness();
  void callback(const geometry_msgs::WrenchStampedConstPtr wrench_ee);

};
#endif // FORCETRACKSTIFFNESS_H
