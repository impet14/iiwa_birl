#ifndef IMPEDANCEPOSITIONCONTROL_H
#define IMPEDANCEPOSITIONCONTROL_H
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include "iiwa_msgs/ControlMode.h"
#include "../iiwa_force_control/ControlModeSwitcher.h"

namespace impedance_position_control {

#define Force_Threshold 0.6

class ImpedancePositionControl : public ControlModeSwithcher
{
private:
  ros::NodeHandle nh_;
  ros::Publisher command_cartesian_pose_pub;
  ros::Subscriber cartesian_wrench_sub;
  int control_mode;
public:
  ImpedancePositionControl(ros::NodeHandle &nh, const int control_mode_ = iiwa_msgs::ControlMode::POSITION_CONTROL);
  ~ImpedancePositionControl();
  void callback(const geometry_msgs::WrenchStampedConstPtr &ft_compensated_cartesian_wrench_);
  void commandCartesianPose(const geometry_msgs::PoseStamped &command_cartesian_pose_);
  void commandCartesianPose(const Eigen::Matrix<float, 7, 1> &command_cartesian_pose_);
  void commandCartesianPose(const Eigen::Matrix<float, 7, 1> &command_cartesian_pose_, const int &time_step_);

};//end class
}//end namespace
#endif IMPEDANCEPOSITIONCONTROL_H


