#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_impedance_position");
  ros::NodeHandle nh;

  ros::Publisher cartesian_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    cartesian_pose_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
