#ifndef CONTROLMODESWITCHER_H
#define CONTROLMODESWITCHER_H

#include <ros/ros.h>
#include "iiwa_msgs/ConfigureSmartServo.h"
#include "iiwa_msgs/ControlMode.h"
#include "iiwa_msgs/DOF.h"

class ControlModeSwitcher{
private:
  ros::NodeHandle nh;
  ros::ServiceClient control_mode_client;

  bool switcher;

public:
  ControlModeSwitcher(ros::NodeHandle &nh_);
  ~ControlModeSwitcher();

  /**
   * @brief call for position control mode
   * @return switcher, if the client called successfully the switcher is true, otherwise is false.
   */
  bool callPositionControl();

  /**
   * @brief call for Cartesian impedance control mode
   * @return switcher, if the client called successfully the switcher is true, otherwise is false.
   */
  bool callCartesianImpedanceControl();

  /**
   * @brief call for joint impedance control mode
   * @return switcher, if the client called successfully the switcher is true, otherwise is false.
   */
  bool callJointImpedanceControl();

  /**
   * @brief call for desired force control mode
   * @return switcher, if the client called successfully the switcher is true, otherwise is false.
   */
  bool callDesiredForceControl();

  /**
   * @brief callSinePatternControlMode
   * @return
   */
  bool callSinePatternControlMode();

};
#endif // CONTROLMODESWITCHER_H
