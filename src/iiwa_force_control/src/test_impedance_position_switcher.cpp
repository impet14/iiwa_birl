#include <ros/ros.h>
#include "../../iiwa_force_control/include/iiwa_force_control/PositionImpedanceSwitcher.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_impedance_position_switcher");
  ros::NodeHandle nh;
  const std::string robot_description = "/iiwa/robot_description";
  iiwa_force_control::PositionImpedanceSwitcher test_switcher(nh, robot_description);
  ROS_INFO("test parellal force and position control");
  int i;
  i = 500;
  //test_switcher.callSinePatternControlMode();
  //test_switcher.callCartesianImpedanceControl();
  //test_switcher.callDesiredForceControl();
  //if(test_switcher.callDesiredForceControl())
  //{
  //  for(i = 200; i>0; i--)
  //  {
  //   test_switcher.commandCartesianPose();
      //sleep(0.1);
  // }
  //}

/*
  while(i>0)
  {
    i--;
    ROS_INFO("KO1");
    test_switcher.commandCartesianPose();
   // sleep(1);
    ROS_INFO("KO2");
    if(test_switcher.callCartesianImpedanceControl())
    {
      //sleep(0.05);
      test_switcher.callPositionControl();
    }
  }
  */
  ros::spin();
  return 0;
}
