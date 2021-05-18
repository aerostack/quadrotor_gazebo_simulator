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

#include <mav_msgs/default_topics.h>

#include <motor_speed_controller/PID_attitude_controller_node.h>

namespace mav_control {

PIDAttitudeControllerNode::PIDAttitudeControllerNode(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      got_first_attitude_command_(false),
      PID_attitude_controller_(nh, private_nh)
{

  PID_attitude_controller_.InitializeParams();

  //Get topics 
  ros::param::get("~self_localization_pose_topic", self_localization_pose_topic_str);
  ros::param::get("~self_localization_speed_topic", self_localization_speed_topic_str);
  ros::param::get("~motor_speed_topic", motor_speed_topic_str);
  ros::param::get("~roll_pitch_yawrate_thrust_topic", roll_pitch_yawrate_thrust_topic_str);

  command_roll_pitch_yawrate_thrust_sub_ = nh_.subscribe(
      roll_pitch_yawrate_thrust_topic_str, 1,
      &PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback, this, ros::TransportHints().tcpNoDelay());

  command_thrust_sub_ = nh_.subscribe(
      "actuator_command/thrust", 1,
      &PIDAttitudeControllerNode::CommandThrustCallback, this, ros::TransportHints().tcpNoDelay());

  command_pose_sub_ = nh_.subscribe(
      "actuator_command/pose", 1,
      &PIDAttitudeControllerNode::CommandPoseCallback, this, ros::TransportHints().tcpNoDelay());

  command_speed_sub_ = nh_.subscribe(
      "actuator_command/speed", 1,
      &PIDAttitudeControllerNode::CommandSpeedCallback, this, ros::TransportHints().tcpNoDelay());
  
  self_localization_pose_sub_ = nh_.subscribe(self_localization_pose_topic_str, 1,
                                &PIDAttitudeControllerNode::selfLocalizationPoseCallback, this,
                                ros::TransportHints().tcpNoDelay());

  self_localization_speed_sub_ = nh_.subscribe(self_localization_speed_topic_str, 1,
                                &PIDAttitudeControllerNode::selfLocalizationSpeedCallback, this,
                                ros::TransportHints().tcpNoDelay());

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      motor_speed_topic_str, 1);

  dynamic_reconfigure::Server<mav_linear_mpc::PIDAttitudeConfig>::CallbackType f;
  f = boost::bind(&PIDAttitudeControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);
  received_pose = false;
}

PIDAttitudeControllerNode::~PIDAttitudeControllerNode()
{
}

void PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference){
  PID_attitude_controller_.SetDesiredAttitude(roll_pitch_yawrate_thrust_reference->roll,
                                              roll_pitch_yawrate_thrust_reference->pitch,
                                              roll_pitch_yawrate_thrust_reference->yaw_rate,
                                              roll_pitch_yawrate_thrust_reference->thrust.z);
  got_first_attitude_command_ = true;
}

void PIDAttitudeControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedPtr& pose_reference){
  tf::Matrix3x3 m(tf::Quaternion(pose_reference->pose.orientation.x, pose_reference->pose.orientation.y, 
  pose_reference->pose.orientation.z, pose_reference->pose.orientation.w));
  double y, p, r;
  m.getEulerYPR(y, p, r);
  roll = r;
  pitch = p;
  yaw = y;
  PID_attitude_controller_.SetDesiredAttitude(r, p, dyaw, last_thrust);
  //PID_attitude_controller_.SetDesiredAttitude(pose_reference->pose.orientation.x, pose_reference->pose.orientation.y, pose_reference->pose.orientation.z, last_thrust/2);
  got_first_attitude_command_ = true;
}

void PIDAttitudeControllerNode::CommandSpeedCallback(
    const geometry_msgs::TwistStampedPtr& speed_reference){
  dpitch = speed_reference->twist.angular.x;
  droll = speed_reference->twist.angular.y;
  dyaw = speed_reference->twist.angular.z;
  //PID_attitude_controller_.SetDesiredAttitude(pose_reference->pose.orientation.x, pose_reference->pose.orientation.y, pose_reference->pose.orientation.z, last_thrust/2);
  got_first_attitude_command_ = true;
}

void PIDAttitudeControllerNode::CommandThrustCallback(
    const mavros_msgs::ThrustPtr& thrust_reference){
  last_thrust = thrust_reference->thrust;
}

void PIDAttitudeControllerNode::selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  self_localization_pose_msg = *msg;
  received_pose = true;
}

void PIDAttitudeControllerNode::selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  if (!received_pose){
    return;
  }
  // WORLDFRAME to BODYFRAME
  self_localization_speed_msg = *msg;

  nav_msgs::Odometry odom_msg;
  tf::Quaternion q(self_localization_pose_msg.pose.orientation.x, self_localization_pose_msg.pose.orientation.y, 
  self_localization_pose_msg.pose.orientation.z, self_localization_pose_msg.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double y, p, r;
  m.getEulerYPR(y, p, r);

  Eigen::Vector3f BodyFrame;
  Eigen::Vector3f GlobalFrame;
  Eigen::Matrix3f RotationMat;

  GlobalFrame(0,0) = (+1)*self_localization_speed_msg.twist.linear.x;
  GlobalFrame(1,0) = (+1)*self_localization_speed_msg.twist.linear.y;
  GlobalFrame(2,0) = 0;

  RotationMat(0,0) = cos(y);
  RotationMat(1,0) = -sin(y);
  RotationMat(2,0) = 0;

  RotationMat(0,1) = sin(y);
  RotationMat(1,1) = cos(y);
  RotationMat(2,1) = 0;

  RotationMat(0,2) = 0;
  RotationMat(1,2) = 0;
  RotationMat(2,2) = 1;

  BodyFrame = RotationMat.transpose().inverse()*GlobalFrame;

  //Publish odometry
  odom_msg.header = self_localization_pose_msg.header;
  odom_msg.pose.pose = self_localization_pose_msg.pose;
  odom_msg.twist.twist = self_localization_speed_msg.twist;
  odom_msg.twist.twist.linear.x = BodyFrame(0);
  odom_msg.twist.twist.linear.y = BodyFrame(1);

  // Get odometry from self_localization topics 
  mav_msgs::EigenOdometry odometry;
  eigenOdometryFromMsg(odom_msg, &odometry);
  this->processOdometry(odometry);
}

void PIDAttitudeControllerNode::processOdometry(mav_msgs::EigenOdometry odometry)
{
  ROS_INFO_ONCE("PIDAttitudeController got first odometry message.");

  if (!got_first_attitude_command_)
    return;

  PID_attitude_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  PID_attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::Actuators turning_velocities_msg;

  turning_velocities_msg.angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg.angular_velocities.push_back(ref_rotor_velocities[i]);
  turning_velocities_msg.header.stamp = self_localization_speed_msg.header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);
}

void PIDAttitudeControllerNode::DynConfigCallback(mav_linear_mpc::PIDAttitudeConfig &config,
                                                  uint32_t level)
{

  PID_attitude_controller_.SetPIDParameters(config.roll_gain, config.pitch_gain, config.p_gain,
                                            config.q_gain, config.r_gain, config.roll_int_gain,
                                            config.pitch_int_gain);
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PIDAttitudeControllerNode");

  ros::NodeHandle nh, private_nh("~");

  mav_control::PIDAttitudeControllerNode PID_attitude_controller(nh, private_nh);

  ros::spin();

  return 0;
}
