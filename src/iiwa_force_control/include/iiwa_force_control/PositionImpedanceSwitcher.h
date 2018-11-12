#ifndef POSITIONIMPEDANCESWITCHER_H
#define POSITIONIMPEDANCESWITCHER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "iiwa_msgs/ConfigureSmartServo.h"
#include "iiwa_msgs/ControlMode.h"
#include "iiwa_msgs/DOF.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointVelocity.h"

#include <Eigen/SVD>
#include <Eigen/Eigen>

namespace iiwa_force_control {

enum{CARTESIAN_IMPEDENCE_CONTROL = 1};

#define FORCE_ERROR 0.8
#define FORCE_COMPENSATION 1.6

typedef message_filters::sync_policies::ApproximateTime<iiwa_msgs::JointPosition, geometry_msgs::WrenchStamped> my_sync_policy;

  class PositionImpedanceSwitcher
  {
  private:
    ///The ros node handle
    ros::NodeHandle nh;
    ///The client can change the control mode
    ros::ServiceClient client;
    ///Publishing the command of the robot end-effector
    ros::Publisher command_pub;
    ///Subscribing the cartesian wrench topic
    ros::Subscriber cartesian_wrench_sub;
    ///The control mode changed successfully or not
    bool switcher;
    ///If the control_mode value is 1, then the robot control mode is cartesian impedance control mode
    uint control_mode;
    ///Moveit objects to calculate the Jacobian of iiwa robot
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_state::RobotStatePtr iiwa_kinematic_state;
    const robot_state::JointModelGroup * iiwa_joint_model_group;
    ///To calculate the Jacobian
    Eigen::Vector3d reference_point_position;
    ///Synchronizing the topics, joint position and cartesian wrench
    message_filters::Subscriber<iiwa_msgs::JointPosition> * iiwa_joint_position_sub;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> * iiwa_wrench_sub;
    //message_filters::TimeSynchronizer<iiwa_msgs::JointPosition, geometry_msgs::PoseArray>  sync;
    message_filters::Synchronizer<my_sync_policy> * sync;
    ///target pose
    geometry_msgs::PoseArray target_pose;
    Eigen::Matrix<float, 6, 1> target_cartesian_pose, target_cartesian_pose1, target_cartesian_pose2;

  public:
    PositionImpedanceSwitcher(ros::NodeHandle &nh_);
    PositionImpedanceSwitcher(ros::NodeHandle &nh_, const std::string &robot_description_);
    ~PositionImpedanceSwitcher();

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

    /**
     * @brief commandCartesianPose
     */
    void commandCartesianPose();

    /**
     * @brief commandJointVelocity
     */
    void callbackJointVelocity(const iiwa_msgs::JointPositionConstPtr iiwa_joint_position_);

    /**
     * @brief callbackContactDetection
     * @param cartesian_wrench_
     */
    void callbackContactDetection(const geometry_msgs::WrenchStampedConstPtr &cartesian_wrench_);

    /**
     * @brief callbackVelocityControl
     * @param iiwa_joint_position_
     * @param iiwa_cartesian_wrench_
     */
    void callbackVelocityControl(const iiwa_msgs::JointPositionConstPtr iiwa_joint_position_, const geometry_msgs::WrenchStampedConstPtr iiwa_cartesian_wrench_);

    /**
     * @brief calculatePseudoInv
     * @param jacobian
     * @return
     */
    Eigen::MatrixXd calculatePseudoInv(Eigen::MatrixXd & jacobian);
  };
}//end namespace

#endif // POSITIONIMPEDANCESWITCHER_H
