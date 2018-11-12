#include "../../iiwa_force_control/include/iiwa_force_control/ForceTrackStiffness.h"

ForceTrackStiffness::ForceTrackStiffness(const ros::NodeHandle &nh_)
  :nh(nh_)
  ,k_p(100)
{
  desired_force<<0, 0, 1, 0, 0, 0;
  config.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
  client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  force_error_pub = nh.advertise<geometry_msgs::WrenchStamped>("/tf_error", 1);
  wrench_sub = nh.subscribe<geometry_msgs::WrenchStampedConstPtr>("/iiwa/state/CartesianWrench", 1, &ForceTrackStiffness::callback, this);
}

ForceTrackStiffness::~ForceTrackStiffness()
{
  // do nothing:-)
}
//force control loop
void ForceTrackStiffness::callback(const geometry_msgs::WrenchStampedConstPtr wrench_ee)
{
  //calculate the force and torque error
  Eigen::Matrix<float, 6, 1> force_error, torque_error;
  force_error(0, 0) = wrench_ee->wrench.force.x - desired_force(0, 0);
  force_error(1, 0) = wrench_ee->wrench.force.y - desired_force(1, 0);
  force_error(2, 0) = wrench_ee->wrench.force.z - desired_force(2, 0);
  torque_error(3, 0) = wrench_ee->wrench.torque.x - desired_force(3, 0);
  torque_error(4, 0) = wrench_ee->wrench.torque.y - desired_force(4, 0);
  torque_error(5, 0) = wrench_ee->wrench.torque.z - desired_force(5, 0);

  //publish the force and torque error
  geometry_msgs::WrenchStamped ft_error;
  ft_error.header.stamp = wrench_ee->header.stamp;
  ft_error.header.frame_id = wrench_ee->header.frame_id;

  ft_error.wrench.force.x = force_error(0, 0);
  ft_error.wrench.force.y = force_error(1, 0);
  ft_error.wrench.force.z = force_error(2, 0);
  ft_error.wrench.torque.x = force_error(3, 0);
  ft_error.wrench.torque.y = force_error(4, 0);
  ft_error.wrench.torque.z = force_error(5, 0);

  force_error_pub.publish(ft_error);

  //PD controller


  // Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad].
  // Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0.
  config.request.cartesian_impedance.cartesian_stiffness.x = 100.0;
  config.request.cartesian_impedance.cartesian_stiffness.y = 100.0;
  config.request.cartesian_impedance.cartesian_stiffness.z = 40.0;
  config.request.cartesian_impedance.cartesian_stiffness.a = 10.0;
  config.request.cartesian_impedance.cartesian_stiffness.b = 10.0;
  config.request.cartesian_impedance.cartesian_stiffness.c = 10.0;

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

    }
    else{
      ROS_INFO_STREAM("SmartServo Service successfully called.");

    }
  }
  else {
    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");

  }
}

