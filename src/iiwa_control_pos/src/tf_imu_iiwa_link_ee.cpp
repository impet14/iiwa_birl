#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Eigen>

const double PI = 3.14159626;
bool flag_calculated = false;

tf::Transform transform;

void callbackFunction(const sensor_msgs::ImuConstPtr &imu_)
{
   static tf::TransformBroadcaster br;
   static tf::TransformBroadcaster br_imu;
  //Only calculate the transformation once, so set it true
  flag_calculated = true;
  //Get the transformation between the imu frame and its world frame
  tf::Transform tf_world_imu;
  tf_world_imu.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setValue(imu_->orientation.x, imu_->orientation.y, imu_->orientation.z, imu_->orientation.w);
  tf_world_imu.setRotation(q);
  br_imu.sendTransform(tf::StampedTransform(tf_world_imu, ros::Time::now(), "imu_world", "imu"));
  std::cout<<"debug"<<std::endl;

  //Get the transform between the robot world frame and the iiwa_link_ee
  tf::TransformListener listener;
  tf::StampedTransform tf_world_robot_ee;
  try{
     listener.waitForTransform("/world", "/iiwa_link_ee", ros::Time(0.0), ros::Duration(1.0));
     listener.lookupTransform("/world", "/iiwa_link_ee", ros::Time(0.0), tf_world_robot_ee);
     }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
     }
  tf_world_robot_ee.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  //Set the transformation between the link_ee frame and the IMU frame
  tf::Transform tf_ee_imu;
  tf_ee_imu.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q_1;
  q_1.setRPY(PI/2, PI/2, 0);
  tf_ee_imu.setRotation(q_1);

  std::cout<<"The transformation between the iiwa_link_ee and the IMU is: "<< tf_ee_imu.getRotation().getX()<<" "
                                                                           << tf_ee_imu.getRotation().getY()<<" "
                                                                           << tf_ee_imu.getRotation().getZ()<<" "
                                                                           << tf_ee_imu.getRotation().getW()<<std::endl;

  //Calculate the transformation between the iiwa_link_ee frame and the IMU frame
  /*
  tf_world_robot_ee.mult(tf_ee_imu, tf_world_robot_ee);
  transform.mult(tf_world_imu.inverse(), tf_world_robot_ee);
  */
  transform.mult(tf_world_robot_ee, tf_ee_imu);
  //tf_world_robot_ee *= tf_ee_imu;
  tf_world_imu.getBasis().transpose();

  //br.sendTransform(tf::StampedTransform(tf_world_imu, ros::Time::now(), "imu_world", "imu"));
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  transform.mult(transform, tf_world_imu);
  //std::cout<<"The transformation between the "<<transform.getBasis().getColumn(0)<<std::endl;

  //transform *= tf_world_imu;
  std::cout<<"The transformation between the iiwa_link_0 and the world frame of the IMU is: "<< transform.getRotation().getX()<<" "
                                                                           << transform.getRotation().getY()<<" "
                                                                           << transform.getRotation().getZ()<<" "
                                                                           << transform.getRotation().getW()<<std::endl;

  //tf::Transform I;
  //I.setIdentity();

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "iiwa_link_0", "imu_world"));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_imu_iiwa_link_ee");
  ros::NodeHandle nh;

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &callbackFunction);
  ROS_INFO("Glad to publish the transformation bewteen the IMU frame and the iiwa_link_ee !");
  //ros::spinOnce();
  ros::spin();
  //Publish the transformation with 100Hz
  /*
  ros::Rate rate(100);
  while(nh.ok())
  {

    if(!flag_calculated)
    {
      ros::spinOnce();
      std::cout<<"Hi, calculating the transformation between the world frame of the robot and the world frame of IMU"<<std::endl;
    }else{
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "iiwa_link_0", "imu_world"));
      rate.sleep();
    }
  }
  */

  return 0;
}
