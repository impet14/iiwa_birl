#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/WrenchStamped.h>
ros::Publisher pub;
void chatterCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  geometry_msgs::WrenchStamped change;
  change.header.stamp = msg->header.stamp;
  /*change.wrench.force.x = msg->wrench.force.x;
  change.wrench.force.y = msg->wrench.force.y;
  change.wrench.force.z = msg->wrench.force.z;
  change.wrench.torque.x = msg->wrench.torque.x;
  change.wrench.torque.y = msg->wrench.torque.y;
  change.wrench.torque.z = msg->wrench.torque.z*/

  change = *msg;
  change.header.frame_id = "iiwa_link_ee";
  pub.publish(change);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_wrench_frame_id");
  ros::NodeHandle nh;

  ros::Subscriber sub_ = nh.subscribe<geometry_msgs::WrenchStamped>("/iiwa/state/CartesianWrench", 1, chatterCallback);
  pub = nh.advertise<geometry_msgs::WrenchStamped>("/iiwa/CartesianWrench", 1);
  ROS_INFO("The wrench frame id has been changed to iiwa_link_ee");
  ros::spin();

  return 0;
}
