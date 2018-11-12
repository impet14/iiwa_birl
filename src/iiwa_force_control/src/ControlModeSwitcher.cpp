#include "../../iiwa_force_control/include/iiwa_force_control/ControlModeSwitcher.h"

ControlModeSwitcher::ControlModeSwitcher(ros::NodeHandle &nh_)
  :nh(nh_)
{
  control_mode_client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
}

ControlModeSwitcher::~ControlModeSwitcher()
{
  //do nothing
}

bool ControlModeSwitcher::callPositionControl()
{
  switcher = false;
  iiwa_msgs::ConfigureSmartServo config;

  config.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;
  if (client.call(config)) {
      if(!config.response.success){
          ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
          switcher = config.response.success;
      }
      else{
          ROS_INFO_STREAM("SmartServo Service successfully called.");
          switcher = config.response.success;
      }
  }
  else {
      ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
      switcher = false;
  }
  return switcher;
}

bool ControlModeSwitcher::callCartesianImpedanceControl()
{
  switcher = false;
  iiwa_msgs::ConfigureSmartServo config;

  config.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE; // This is equal to config.control_mode = 2.

  // Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad].
  // Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0.
  config.request.cartesian_impedance.cartesian_stiffness.x = 30.0;
  config.request.cartesian_impedance.cartesian_stiffness.y = 30.0;
  config.request.cartesian_impedance.cartesian_stiffness.z = 1000.0;
  config.request.cartesian_impedance.cartesian_stiffness.a = 100.0;
  config.request.cartesian_impedance.cartesian_stiffness.b = 100.0;
  config.request.cartesian_impedance.cartesian_stiffness.c = 100.0;

  // Dimensionless damping values for the cartesian impedance control, for all degrees of freedom : [x, y, z, a, b, c].
  // Precondition: 0.1 <= x, y, z, a, b, c <= 1.0.
  config.request.cartesian_impedance.cartesian_damping.x = 1;
  config.request.cartesian_impedance.cartesian_damping.y = 1;
  config.request.cartesian_impedance.cartesian_damping.z = 1;
  config.request.cartesian_impedance.cartesian_damping.a = 1;
  config.request.cartesian_impedance.cartesian_damping.b = 1;
  config.request.cartesian_impedance.cartesian_damping.c = 1;

  // The stiffness value for null space [Nm/rad].
  // Precondition: 0.0 <= value.
  config.request.cartesian_impedance.nullspace_stiffness = 0.5;

  // The damping parameter for null space [Nm*s/rad].
  // Precondition: value >= 0.3 and value <= 1.0. - A good damping value is 0.7.
  config.request.cartesian_impedance.nullspace_damping = 0.7; // stiffness to 100 N/m

  if (client.call(config)) {
      if(!config.response.success){
          ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
          switcher = config.response.success;
      }
      else{
          ROS_INFO_STREAM("SmartServo Service successfully called.");
          switcher = config.response.success;
      }
  }
  else {
      ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
      switcher = false;
  }
  return switcher;
}

bool ControlModeSwitcher::callJointImpedanceControl()
{
  switcher = false;
  iiwa_msgs::ConfigureSmartServo config;

  config.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE; // This is equal to config.control_mode = 1.

  // Stiffness values in [Nm/rad]. Stiffness values must be >= 0.
  config.request.joint_impedance.joint_stiffness.a1 = 0.0;
  config.request.joint_impedance.joint_stiffness.a2 = 0.0;
  config.request.joint_impedance.joint_stiffness.a3 = 0.0;
  config.request.joint_impedance.joint_stiffness.a4 = 0.0;
  config.request.joint_impedance.joint_stiffness.a5 = 0.0;
  config.request.joint_impedance.joint_stiffness.a6 = 0.0;
  config.request.joint_impedance.joint_stiffness.a7 = 0.0;

  // Damping values. Damping values must be between 0 and 1.
  config.request.joint_impedance.joint_damping.a1 = 1;
  config.request.joint_impedance.joint_damping.a2 = 1;
  config.request.joint_impedance.joint_damping.a3 = 1;
  config.request.joint_impedance.joint_damping.a4 = 1;
  config.request.joint_impedance.joint_damping.a5 = 1;
  config.request.joint_impedance.joint_damping.a6 = 1;
  config.request.joint_impedance.joint_damping.a7 = 1;

  if (client.call(config)) {
      if(!config.response.success){
          ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
          switcher = config.response.success;
      }
      else{
          ROS_INFO_STREAM("SmartServo Service successfully called.");
          switcher = config.response.success;
      }
  }
  else {
      ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
      switcher = false;
  }
  return switcher;
}

bool ControlModeSwitcher::callDesiredForceControl()
{
  switcher = false;
  iiwa_msgs::ConfigureSmartServo config;

  config.request.control_mode = iiwa_msgs::ControlMode::DESIRED_FORCE; // This is equal to config.control_mode = 3.
  config.request.desired_force. cartesian_dof = iiwa_msgs::DOF::Z; // This is equal to config.desired_force.cartesian_dof = 3.
  config.request.desired_force. desired_force = 2.0; // 2 Newtons
  config.request.desired_force. desired_stiffness = 100; // stiffness to 100 N/m

  if (client.call(config)) {
      if(!config.response.success){
          ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
          switcher = config.response.success;
      }
      else{
          ROS_INFO_STREAM("SmartServo Service successfully called.");
          switcher = config.response.success;
      }
  }
  else {
      ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
      switcher = false;
  }
  return switcher;
}

bool PositionImpedanceSwitcher::callSinePatternControlMode()
{
  switcher = false;
  iiwa_msgs::ConfigureSmartServo config;

  config.request.control_mode = iiwa_msgs::ControlMode::SINE_PATTERN;
  config.request.sine_pattern.cartesian_dof = 2;
  config.request.sine_pattern.frequency = 1;
  config.request.sine_pattern.amplitude = 10;
  config.request.sine_pattern.stiffness = 10;// stiffness to 100 N/m

  if (client.call(config)) {
      if(!config.response.success){
          ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
          switcher = config.response.success;
      }
      else{
          ROS_INFO_STREAM("SmartServo Service successfully called.");
          switcher = config.response.success;
      }
  }
  else {
      ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
      switcher = false;
  }
  return switcher;
}
