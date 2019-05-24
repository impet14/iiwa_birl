  #include "../../iiwa_force_control/include/iiwa_force_control/PositionImpedanceSwitcher.h"

namespace iiwa_force_control {
/*
PositionImpedanceSwitcher::PositionImpedanceSwitcher(ros::NodeHandle &nh_)
  :nh(nh_)
  ,control_mode(0)
{
  client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  command_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1);
  cartesian_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/iiwa/state/CartesianWrench", 1, &iiwa_force_control::PositionImpedanceSwitcher::callbackContactDetection, this);
  iiwa_joint_position_sub = nullptr;
  iiwa_wrench_sub = nullptr;
  sync = nullptr;
}
*/
PositionImpedanceSwitcher::PositionImpedanceSwitcher(ros::NodeHandle &nh_, const std::string &robot_description_)
  :nh(nh_)
  ,control_mode(0) //check?
  ,reference_point_position(0, 0, 0) //check?
  ,robot_model_loader(robot_description_)
  ,iiwa_kinematic_state(new robot_state::RobotState(robot_model_loader.getModel()))
  ,iiwa_joint_model_group(robot_model_loader.getModel()->getJointModelGroup("manipulator"))
{
  //set target points
  // use global variable to read the saved pose coming from geometry_msgs/Pose
  target_cartesian_pose1 << 0.63465,  0.06999, 0.22461, 0, 0, 0;
  target_cartesian_pose2 << 0.62219, -0.06589, 0.22461, 0, 0, 0;
  target_cartesian_pose = target_cartesian_pose1;
  //no sync

  // Publishers and Subscribers
  cartesian_wrench_sub = nh.subscribe<iiwa_msgs::JointPositionConstPtr>("/iiwa/state/JointPosition", 1, &PositionImpedanceSwitcher::callbackJointVelocity, this);
  command_pub = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1);

  // Services
  client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");

  //sync the joint position topic and cartesian wrench
  /*
  iiwa_joint_position_sub = new message_filters::Subscriber<iiwa_msgs::JointPosition>(nh, "/iiwa/state/JointPosition", 1);
  iiwa_wrench_sub = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh, "/iiwa/state/CartesianWrench", 1);
  sync = new message_filters::Synchronizer<my_sync_policy>(my_sync_policy(5), *iiwa_joint_position_sub, *iiwa_wrench_sub);
  //sync = new message_filters::TimeSynchronizer<iiwa_msgs::JointPosition, geometry_msgs::PoseArray>(iiwa_joint_position_sub, iiwa_endeffector_link_command_cartsian_vel_sub, 5);
  sync->registerCallback(boost::bind(&PositionImpedanceSwitcher::callbackVelocityControl, this, _1, _2));
  */

}



PositionImpedanceSwitcher::~PositionImpedanceSwitcher()
{
  /*
  delete iiwa_joint_position_sub;
  delete iiwa_wrench_sub;
  delete sync;
  */
}

bool PositionImpedanceSwitcher::callPositionControl()
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

bool PositionImpedanceSwitcher::callCartesianImpedanceControl()
{
  switcher = false;
  iiwa_msgs::ConfigureSmartServo config;

  config.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE; // This is equal to config.control_mode = 2.

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

bool PositionImpedanceSwitcher::callJointImpedanceControl()
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

bool PositionImpedanceSwitcher::callDesiredForceControl()
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

void PositionImpedanceSwitcher::commandCartesianPose()
{
  ROS_INFO("KO3");
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
  ROS_INFO("KO4");
  geometry_msgs::PoseStamped end_effecotor_command_pose;
  end_effecotor_command_pose.header.frame_id = "iiwa_link_0";
  end_effecotor_command_pose.header.stamp = ros::Time::now();
  std::cout<<base_end_eff_transform.getOrigin().getX()<<std::endl;
  end_effecotor_command_pose.pose.position.x = base_end_eff_transform.getOrigin().getX();
  end_effecotor_command_pose.pose.position.y = 0.007 + base_end_eff_transform.getOrigin().getY();
  end_effecotor_command_pose.pose.position.z = base_end_eff_transform.getOrigin().getZ();
  end_effecotor_command_pose.pose.orientation.x = base_end_eff_transform.getRotation().getX();
  end_effecotor_command_pose.pose.orientation.y = base_end_eff_transform.getRotation().getY();
  end_effecotor_command_pose.pose.orientation.z = base_end_eff_transform.getRotation().getZ();
  end_effecotor_command_pose.pose.orientation.w = base_end_eff_transform.getRotation().getW();
  ROS_INFO("KO5");
  command_pub.publish(end_effecotor_command_pose);
}

void PositionImpedanceSwitcher::callbackJointVelocity(const iiwa_msgs::JointPositionConstPtr iiwa_joint_position_)
{
  //switch the robot mode into cartesian impedance control mode
  if(control_mode != CARTESIAN_IMPEDENCE_CONTROL)
  {
    callCartesianImpedanceControl();
    control_mode = CARTESIAN_IMPEDENCE_CONTROL;
  }

  //get the current joint state
  Eigen::VectorXd joint_state(7);
  joint_state << iiwa_joint_position_->position.a1,
      iiwa_joint_position_->position.a2,
      iiwa_joint_position_->position.a3,
      iiwa_joint_position_->position.a4,
      iiwa_joint_position_->position.a5,
      iiwa_joint_position_->position.a6,
      iiwa_joint_position_->position.a7;
  //set the robot state to the current state for calculating the jacobian
  iiwa_kinematic_state -> setJointGroupPositions(iiwa_joint_model_group, joint_state);

  //calculate the jacobian by Moveit API
  Eigen::MatrixXd iiwa_jacobian(6, iiwa_joint_model_group -> getVariableCount());

  if(iiwa_kinematic_state -> getJacobian(iiwa_joint_model_group, iiwa_kinematic_state->getLinkModel(iiwa_joint_model_group->getLinkModelNames().back()),reference_point_position, iiwa_jacobian))
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
    Eigen::Matrix<double, 6, 1> command_cartsian_vel;
    command_cartsian_vel << target_cartesian_pose(0,0) - base_end_eff_transform.getOrigin().getX(),
                            target_cartesian_pose(1,0) - base_end_eff_transform.getOrigin().getY(),
                            target_cartesian_pose(2,0) - base_end_eff_transform.getOrigin().getZ(),
                            0, 0, 0;
    if (command_cartsian_vel.norm() > 0.01)
    {
      command_cartsian_vel = 0.02*command_cartsian_vel/command_cartsian_vel.norm();
    }else{
      command_cartsian_vel << 0, 0, 0, 0, 0, 0;
      target_cartesian_pose = target_cartesian_pose2;
      command_cartsian_vel << target_cartesian_pose(0,0) - base_end_eff_transform.getOrigin().getX(),
                              target_cartesian_pose(1,0) - base_end_eff_transform.getOrigin().getY(),
                              target_cartesian_pose(2,0) - base_end_eff_transform.getOrigin().getZ(),
                              0, 0, 0;
      std::cout << "hi, I am here!"<< target_cartesian_pose <<std::endl;
      if(command_cartsian_vel.norm() < 0.01)
      {
        target_cartesian_pose = target_cartesian_pose1;
      }
    }

    Eigen::MatrixXd pinv_iiwa_jacobian = calculatePseudoInv(iiwa_jacobian);

    Eigen::Matrix<double, 7, 1> command_joint_vel_;

    command_joint_vel_ = pinv_iiwa_jacobian * command_cartsian_vel;
    std::cout<<"command velocity is: "<<std::endl<<command_joint_vel_<<std::endl;
    iiwa_msgs::JointVelocity command_joint_vel;

    command_joint_vel.header.stamp = ros::Time::now();
    command_joint_vel.velocity.a1 = command_joint_vel_(0,0);
    command_joint_vel.velocity.a2 = command_joint_vel_(1,0);
    command_joint_vel.velocity.a3 = command_joint_vel_(2,0);
    command_joint_vel.velocity.a4 = command_joint_vel_(3,0);
    command_joint_vel.velocity.a5 = command_joint_vel_(4,0);
    command_joint_vel.velocity.a6 = command_joint_vel_(5,0);
    command_joint_vel.velocity.a7 = command_joint_vel_(6,0);

    command_pub.publish(command_joint_vel);
  }else{

    iiwa_msgs::JointVelocity command_joint_vel;
    command_joint_vel.header.stamp = ros::Time::now();
    command_joint_vel.velocity.a1 = 0.0;
    command_joint_vel.velocity.a2 = 0.0;
    command_joint_vel.velocity.a3 = 0.0;
    command_joint_vel.velocity.a4 = 0.0;
    command_joint_vel.velocity.a5 = 0.0;
    command_joint_vel.velocity.a6 = 0.0;
    command_joint_vel.velocity.a7 = 0.0;
  }
}

void PositionImpedanceSwitcher::callbackContactDetection(const geometry_msgs::WrenchStampedConstPtr &cartesian_wrench_)
{
  double outer_force;
  outer_force = sqrt(pow(cartesian_wrench_->wrench.force.x, 2) + pow(cartesian_wrench_->wrench.force.y, 2) + pow(cartesian_wrench_->wrench.force.z, 2));
  std::cout<<outer_force<<std::endl;
  if((outer_force - FORCE_COMPENSATION) > FORCE_ERROR)
  {
    if(control_mode != CARTESIAN_IMPEDENCE_CONTROL)
    {
      callCartesianImpedanceControl();
      control_mode = CARTESIAN_IMPEDENCE_CONTROL;
    }
  }else{
    commandCartesianPose();
  }
}

void PositionImpedanceSwitcher::callbackVelocityControl(const iiwa_msgs::JointPositionConstPtr iiwa_joint_position_, const geometry_msgs::WrenchStampedConstPtr iiwa_cartesian_wrench_)
{
  if(control_mode != CARTESIAN_IMPEDENCE_CONTROL)
  {
    callCartesianImpedanceControl();
    control_mode = CARTESIAN_IMPEDENCE_CONTROL;
  }

  Eigen::VectorXd joint_state(7);
  joint_state <<iiwa_joint_position_->position.a1,
                iiwa_joint_position_->position.a2,
                iiwa_joint_position_->position.a3,
                iiwa_joint_position_->position.a4,
                iiwa_joint_position_->position.a5,
                iiwa_joint_position_->position.a6,
                iiwa_joint_position_->position.a7;

  iiwa_kinematic_state -> setJointGroupPositions(iiwa_joint_model_group, joint_state);

  Eigen::MatrixXd iiwa_jacobian(6, iiwa_joint_model_group -> getVariableCount());

  if(iiwa_kinematic_state -> getJacobian(iiwa_joint_model_group, iiwa_kinematic_state->getLinkModel(iiwa_joint_model_group->getLinkModelNames().back()),reference_point_position, iiwa_jacobian))
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
    Eigen::Matrix<double, 6, 1> command_cartsian_vel;
    command_cartsian_vel << target_cartesian_pose(0,0) - base_end_eff_transform.getOrigin().getX(),
                            target_cartesian_pose(1,0) - base_end_eff_transform.getOrigin().getY(),
                            target_cartesian_pose(2,0) - base_end_eff_transform.getOrigin().getZ(),
                            0, 0, 0;
    if (command_cartsian_vel.norm() > 0.01)
    {
      command_cartsian_vel = 0.02*command_cartsian_vel/command_cartsian_vel.norm();
    }else{
      command_cartsian_vel << 0, 0, 0, 0, 0, 0;
      target_cartesian_pose = target_cartesian_pose2;
      command_cartsian_vel << target_cartesian_pose(0,0) - base_end_eff_transform.getOrigin().getX(),
                              target_cartesian_pose(1,0) - base_end_eff_transform.getOrigin().getY(),
                              target_cartesian_pose(2,0) - base_end_eff_transform.getOrigin().getZ(),
                              0, 0, 0;
      std::cout << "hi, I am here!"<< target_cartesian_pose <<std::endl;
      if(command_cartsian_vel.norm() < 0.01)
      {
        target_cartesian_pose = target_cartesian_pose1;
      }
    }

    Eigen::MatrixXd pinv_iiwa_jacobian = calculatePseudoInv(iiwa_jacobian);

    Eigen::Matrix<double, 7, 1> command_joint_vel_;

    command_joint_vel_ = pinv_iiwa_jacobian * command_cartsian_vel;
    std::cout<<"command velocity is: "<<std::endl<<command_joint_vel_<<std::endl;
    iiwa_msgs::JointVelocity command_joint_vel;

    command_joint_vel.header.stamp = ros::Time::now();
    command_joint_vel.velocity.a1 = command_joint_vel_(0,0);
    command_joint_vel.velocity.a2 = command_joint_vel_(1,0);
    command_joint_vel.velocity.a3 = command_joint_vel_(2,0);
    command_joint_vel.velocity.a4 = command_joint_vel_(3,0);
    command_joint_vel.velocity.a5 = command_joint_vel_(4,0);
    command_joint_vel.velocity.a6 = command_joint_vel_(5,0);
    command_joint_vel.velocity.a7 = command_joint_vel_(6,0);

    command_pub.publish(command_joint_vel);
  }else{

    iiwa_msgs::JointVelocity command_joint_vel;
    command_joint_vel.header.stamp = ros::Time::now();
    command_joint_vel.velocity.a1 = 0.0;
    command_joint_vel.velocity.a2 = 0.0;
    command_joint_vel.velocity.a3 = 0.0;
    command_joint_vel.velocity.a4 = 0.0;
    command_joint_vel.velocity.a5 = 0.0;
    command_joint_vel.velocity.a6 = 0.0;
    command_joint_vel.velocity.a7 = 0.0;
  }
}

Eigen::MatrixXd PositionImpedanceSwitcher::calculatePseudoInv(Eigen::MatrixXd &jacobian)
{
  //do the jacobiann Moore-penrose pseudo-inverse
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_jac(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd U = svd_jac.matrixU();
  const Eigen::MatrixXd V = svd_jac.matrixV();
  const Eigen::VectorXd S = svd_jac.singularValues();

  Eigen::VectorXd Sing_val = S;
  static const double pinvtoler = std::numeric_limits<float>::epsilon();
  double max_sing_val = 0.0;
  for(std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i){
    if(fabs(S(i)) > max_sing_val){
      max_sing_val = fabs(S(i));
    }
  }
  for(std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i){
    //Those singular value smaller than a percentage of the maximum singular value are removed
    if(fabs(S(i)) > max_sing_val * pinvtoler){
      Sing_val(i) = 1.0 / S(i);
    }else{
      Sing_val(i) = 0.0;
    }
  }
  Eigen::MatrixXd pinv_jacobian = (V * Sing_val.asDiagonal() * U.transpose());
  return pinv_jacobian;
}
}//end namespace

