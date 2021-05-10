/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <stdio.h>
#include <memory.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/Thrust.h"
#include <motor_speed_controller/PIDAttitudeConfig.h>
#include <motor_speed_controller/PID_attitude_controller.h>

namespace mav_control {

class PIDAttitudeControllerNode
{
 public:
  PIDAttitudeControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle private_nh);
  ~PIDAttitudeControllerNode();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  PIDAttitudeController PID_attitude_controller_;

  //publishers
  ros::Publisher motor_velocity_reference_pub_;

  // subscribers
  ros::Subscriber command_roll_pitch_yawrate_thrust_sub_;
  ros::Subscriber command_thrust_sub_;
  ros::Subscriber command_pose_sub_;
  ros::Subscriber command_speed_sub_;
  ros::Subscriber self_localization_pose_sub_;
  ros::Subscriber self_localization_speed_sub_;

  std::string self_localization_pose_topic_str;
  std::string self_localization_speed_topic_str;
  std::string motor_speed_topic_str;
  std::string roll_pitch_yawrate_thrust_topic_str;

  float last_thrust = 0;
  bool received_pose;
  float roll,pitch,yaw,droll,dpitch,dyaw;
  geometry_msgs::PoseStamped self_localization_pose_msg;
  geometry_msgs::TwistStamped self_localization_speed_msg;

  bool got_first_attitude_command_;

  void CommandRollPitchYawRateThrustCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference);
  void CommandThrustCallback(const mavros_msgs::ThrustPtr& thrust_reference);
  void CommandPoseCallback(const geometry_msgs::PoseStampedPtr& pose_reference);
  void CommandSpeedCallback(const geometry_msgs::TwistStampedPtr& speed_reference);
  
  void processOdometry(mav_msgs::EigenOdometry odometry);

  void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  void DynConfigCallback(mav_linear_mpc::PIDAttitudeConfig &config, uint32_t level);

  dynamic_reconfigure::Server<mav_linear_mpc::PIDAttitudeConfig> dyn_config_server_;

};

}
