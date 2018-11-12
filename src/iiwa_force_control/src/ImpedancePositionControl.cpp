#include "../../iiwa_force_control/include/iiwa_force_control/ImpedancePositionControl.h"

namespace impedance_position_control {
ImpedancePositionControl::ImpedancePositionControl(ros::NodeHandle &nh, const int control_mode_ = iiwa_msgs::ControlMode::POSITION_CONTROL)
  :ControlModeSwitcher(nh)
  ,nh_(nh)
  ,control_mode(control_mode_)
{
  if(control_mode != iiwa_msgs::ControlMode::POSITION_CONTROL)
  {
    switch(control_mode)
    case iiwa_msgs::ControlMode::JOINT_IMPEDANCE:
      ControlModeSwitcher::callJointImpedanceControl();
      break;
    case iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE:
      ControlModeSwitcher::callCartesianImpedanceControl();
      break;
    case iiwa_msgs::ControlMode::DESIRED_FORCE:
      ControlModeSwitcher::callDesiredForceControl();
      break;
    case iiwa_msgs::ControlMode::SINE_PATTERN:
      ControlModeSwitcher::callSinePatternControlMode();
      break;
  }
  command_cartesian_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1);
  cartesian_wrench_sub = nh_.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor/ft_compensated", 1, &ImpedancePositionControl::callback, this);
}

void ImpedancePositionControl::callback(const geometry_msgs::WrenchStampedConstPtr &ft_compensated_cartesian_wrench_)
{
  //contact detection
  double contact_force;
  contact_force = sqrt(pow(ft_compensated_cartesian_wrench_->wrench.force.x, 2)
                     + pow(ft_compensated_cartesian_wrench_->wrench.force.y, 2)
                     + pow(ft_compensated_cartesian_wrench_->wrench.force.z, 2));
  std::cout<<contact_force<<std::endl;
  if(contact_force > Force_Threshold){
    if(control_mode != iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE)
    {
     if(ControlModeSwitcher::callCartesianImpedanceControl()){
      control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
      }else{

     }
    }
  }
  commandCartesianPose();
}

void ImpedancePositionControl::commandCartesianPose(const geometry_msgs::PoseStamped &command_cartesian_pose_)
{
  command_cartesian_pose_pub.publish(command_cartesian_pose_);
}

void ImpedancePositionControl::commandCartesianPose(const Eigen::Matrix<float, 7, 1> &command_cartesian_pose_)
{
  geometry_msgs::PoseStamped end_effecotor_command_pose;

  end_effecotor_command_pose.header.frame_id = "iiwa_link_0";
  end_effecotor_command_pose.header.stamp = ros::Time::now();
  end_effecotor_command_pose.pose.position.x = command_cartesian_pose_(0,0);
  end_effecotor_command_pose.pose.position.y = command_cartesian_pose_(1,0);
  end_effecotor_command_pose.pose.position.z = command_cartesian_pose_(2,0);
  end_effecotor_command_pose.pose.orientation.x = command_cartesian_pose_(3,0);
  end_effecotor_command_pose.pose.orientation.y = command_cartesian_pose_(4,0);
  end_effecotor_command_pose.pose.orientation.z = command_cartesian_pose_(5,0);
  end_effecotor_command_pose.pose.orientation.w = command_cartesian_pose_(6,0);

  command_cartesian_pose_pub.publish(end_effecotor_command_pose);
}

void ImpedancePositionControl::commandCartesianPose(const Eigen::Matrix<float, 7, 1> &command_cartesian_pose_, const int &time_step_)
{
  tf::TransformListener tf_listener_;
  tf::StampedTransform base_end_eff_transform;
  try{
    tf_listener_.waitForTransform("iiwa_link_0", "iiwa_link_ee", ros::Time(0.0), ros::Duration(1.0));
    tf_listener_.lookupTransform("iiwa_link_0", "iiwa_link_ee", ros::Time(0.0), base_end_eff_transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  Eigen::Vector3f error(command_cartesian_pose_(0,0) - base_end_eff_transform.getOrigin().getX(),
                        command_cartesian_pose_(1,0) - base_end_eff_transform.getOrigin().getY(),
                        command_cartesian_pose_(2,0) - base_end_eff_transform.getOrigin().getZ());
  if (error.norm() > 0.04)
  {
    error = error/error.norm();
  }else if(error.norm() < 0.01){
    error << 0,0,0;
  }

  geometry_msgs::PoseStamped end_effecotor_command_pose;
  end_effecotor_command_pose.header.frame_id = "iiwa_link_0";
  end_effecotor_command_pose.header.stamp = ros::Time::now();
  end_effecotor_command_pose.pose.position.x = base_end_eff_transform.getOrigin().getX() + error(0,0)*time_step_;
  end_effecotor_command_pose.pose.position.y = base_end_eff_transform.getOrigin().getY() + error(1,0)*time_step_;
  end_effecotor_command_pose.pose.position.z = base_end_eff_transform.getOrigin().getZ() + error(2,0)*time_step_;
  end_effecotor_command_pose.pose.orientation.x = base_end_eff_transform.getRotation().getX();
  end_effecotor_command_pose.pose.orientation.y = base_end_eff_transform.getRotation().getY();
  end_effecotor_command_pose.pose.orientation.z = base_end_eff_transform.getRotation().getZ();
  end_effecotor_command_pose.pose.orientation.w = base_end_eff_transform.getRotation().getW();

  command_cartesian_pose_pub.publish(end_effecotor_command_pose);
}
}//end namespace
