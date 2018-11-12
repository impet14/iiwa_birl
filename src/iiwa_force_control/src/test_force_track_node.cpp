#include <ros/ros.h>
#include "../../iiwa_force_control/include/iiwa_force_control/ForceTrackStiffness.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_force_track_node");
  ros::NodeHandle nh;
  ForceTrackStiffness force_track(nh);

  ROS_INFO("Tracking the desired force now!");
  ros::spin();
}
