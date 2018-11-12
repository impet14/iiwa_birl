#include "ros/ros.h"
#include "iiwa_msgs/ConfigureSmartServo.h"
#include "geometry_msgs/PoseStamped.h"
#include "iiwa_msgs/ControlMode.h"
#include "iiwa_msgs/DOF.h"


/*
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}*/



int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle nh;
  /*
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();
*/

  ros::ServiceClient client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  ros::Publisher command_pose = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",1);

  iiwa_msgs::ConfigureSmartServo config;

  /*position control mode*/
  //config.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;
  config.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE;
  config.request.joint_impedance.joint_stiffness.a1 = 0.0;
  config.request.joint_impedance.joint_stiffness.a2 = 0.0;
  config.request.joint_impedance.joint_stiffness.a3 = 0.0;
  config.request.joint_impedance.joint_stiffness.a4 = 0.0;
  config.request.joint_impedance.joint_stiffness.a5 = 0.0;
  config.request.joint_impedance.joint_stiffness.a6 = 0.0;
  config.request.joint_impedance.joint_stiffness.a7 = 0.0;

  config.request.joint_impedance.joint_damping.a1 = 0.7;
  config.request.joint_impedance.joint_damping.a2 = 0.7;
  config.request.joint_impedance.joint_damping.a3 = 0.7;
  config.request.joint_impedance.joint_damping.a4 = 0.7;
  config.request.joint_impedance.joint_damping.a5 = 0.7;
  config.request.joint_impedance.joint_damping.a6 = 0.7;
  config.request.joint_impedance.joint_damping.a7 = 0.7;

  if (client.call(config)) {
    if(!config.response.success)
      ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
    else
      ROS_INFO_STREAM("SmartServo Service successfully called, the mode is position mode.");
  }
  else {
    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
  }
  /*
  geometry_msgs::PoseStamped com_pose;
        com_pose.header.frame_id = "iiwa_link_0";
  com_pose.header.stamp = ros::Time::now();
  com_pose.pose.position.x = 0.488594140606  ;
  com_pose.pose.position.y = 0.000498421024878;
  com_pose.pose.position.z = 0.507394775102;
  com_pose.pose.orientation.x = 0;
  com_pose.pose.orientation.y = 1;
  com_pose.pose.orientation.z = 0;
  com_pose.pose.orientation.w = 0;
  command_pose.publish(com_pose);
  sleep(4);
  com_pose.header.frame_id = "iiwa_link_0";
  com_pose.header.stamp = ros::Time::now();
  com_pose.pose.position.x = 0.488594140606  ;
  com_pose.pose.position.y = 0.000498421024878;
  com_pose.pose.position.z = 0.307394775102;
  com_pose.pose.orientation.x = 0;
  com_pose.pose.orientation.y = 1;
  com_pose.pose.orientation.z = 0;
  com_pose.pose.orientation.w = 0;
  command_pose.publish(com_pose);
  */

  /*desired force mode*/
  /*	config.request.control_mode = iiwa_msgs::ControlMode::DESIRED_FORCE; // This is equal to config.control_mode = 3.
  config.request.desired_force. cartesian_dof = iiwa_msgs:DOF::Z; // This is equal to config.desired_force.cartesian_dof = 3.*/
  config.request.desired_force.desired_force = 5.0; // 2 Newtons
  config.request.desired_force.desired_stiffness = 10; // stiffness to 100 N/m

  if (client.call(config)) {
    if(!config.response.success)
      ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
    else
      ROS_INFO_STREAM("SmartServo Service successfully called.");
  }
  else {
    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
  }



  return 0;
}
