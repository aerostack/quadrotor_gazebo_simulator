/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_gazebo_plugins/gazebo_wind_gust_plugin.h"

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboWindGustPlugin::~GazeboWindGustPlugin() {
    //  Trying to adapt to Gazebo9
    //  event::Events::ConnectWorldUpdateEnd(update_connection_);
}

void GazeboWindGustPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start0 = kDefaultWindGustStart0;
  double wind_gust_duration0 = kDefaultWindGustDuration0;
  double wind_gust_start1 = kDefaultWindGustStart1;
  double wind_gust_duration1 = kDefaultWindGustDuration1;
  double wind_gust_start2 = kDefaultWindGustStart2;
  double wind_gust_duration2 = kDefaultWindGustDuration2;
  double wind_gust_start3 = kDefaultWindGustStart3;
  double wind_gust_duration3 = kDefaultWindGustDuration3;
  double wind_gust_start4 = kDefaultWindGustStart4;
  double wind_gust_duration4 = kDefaultWindGustDuration4;
  double wind_gust_start5 = kDefaultWindGustStart5;
  double wind_gust_duration5 = kDefaultWindGustDuration5;

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_gust_plugin] Please specify a robotNamespace.\n";

  // Create Gazebo Node.
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initisalise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();
  else
    gzerr << "[gazebo_wind_gust_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windForcePubTopic", wind_force_pub_topic_,
                           wind_force_pub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_,
                           wind_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind params from SDF.
  getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_,
                      wind_force_mean_);
  getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_,
                      wind_force_variance_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windDirection", wind_direction_,
                             wind_direction_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(_sdf, "windGustStart0", wind_gust_start0, wind_gust_start0);
  getSdfParam<double>(_sdf, "windGustDuration0", wind_gust_duration0,
                      wind_gust_duration0);
  getSdfParam<double>(_sdf, "windGustStart1", wind_gust_start1, wind_gust_start1);
  getSdfParam<double>(_sdf, "windGustDuration1", wind_gust_duration1,
                      wind_gust_duration1);
  getSdfParam<double>(_sdf, "windGustStart2", wind_gust_start2, wind_gust_start2);
  getSdfParam<double>(_sdf, "windGustDuration2", wind_gust_duration2,
                      wind_gust_duration2);
  getSdfParam<double>(_sdf, "windGustStart3", wind_gust_start3, wind_gust_start3);
  getSdfParam<double>(_sdf, "windGustDuration3", wind_gust_duration3,
                      wind_gust_duration3);
  getSdfParam<double>(_sdf, "windGustStart4", wind_gust_start4, wind_gust_start4);
  getSdfParam<double>(_sdf, "windGustDuration4", wind_gust_duration4,
                      wind_gust_duration4);
  getSdfParam<double>(_sdf, "windGustStart5", wind_gust_start5, wind_gust_start5);
  getSdfParam<double>(_sdf, "windGustDuration5", wind_gust_duration5,
                      wind_gust_duration5);

  getSdfParam<double>(_sdf, "windGustForceMean0", wind_gust_force_mean0_,
                      wind_gust_force_mean0_);
  getSdfParam<double>(_sdf, "windGustForceVariance0", wind_gust_force_variance0_,
                      wind_gust_force_variance0_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirection0", wind_gust_direction0_,
                             wind_gust_direction0_);
  getSdfParam<double>(_sdf, "windGustForceMean1", wind_gust_force_mean1_,
                      wind_gust_force_mean1_);
  getSdfParam<double>(_sdf, "windGustForceVariance1", wind_gust_force_variance1_,
                      wind_gust_force_variance1_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirection1", wind_gust_direction1_,
                             wind_gust_direction1_);
  getSdfParam<double>(_sdf, "windGustForceMean2", wind_gust_force_mean2_,
                      wind_gust_force_mean2_);
  getSdfParam<double>(_sdf, "windGustForceVariance2", wind_gust_force_variance2_,
                      wind_gust_force_variance2_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirection2", wind_gust_direction2_,
                             wind_gust_direction2_);
  getSdfParam<double>(_sdf, "windGustForceMean3", wind_gust_force_mean3_,
                      wind_gust_force_mean3_);
  getSdfParam<double>(_sdf, "windGustForceVariance3", wind_gust_force_variance3_,
                      wind_gust_force_variance3_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirection3", wind_gust_direction3_,
                             wind_gust_direction3_);
  getSdfParam<double>(_sdf, "windGustForceMean4", wind_gust_force_mean4_,
                      wind_gust_force_mean4_);
  getSdfParam<double>(_sdf, "windGustForceVariance4", wind_gust_force_variance4_,
                      wind_gust_force_variance4_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirection4", wind_gust_direction4_,
                             wind_gust_direction4_);
  getSdfParam<double>(_sdf, "windGustForceMean5", wind_gust_force_mean5_,
                      wind_gust_force_mean5_);
  getSdfParam<double>(_sdf, "windGustForceVariance5", wind_gust_force_variance5_,
                      wind_gust_force_variance5_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirection5", wind_gust_direction5_,
                             wind_gust_direction5_);

  // Get the wind speed params from SDF.
  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_,
                      wind_speed_mean_);
  getSdfParam<double>(_sdf, "windSpeedVariance", wind_speed_variance_,
                      wind_speed_variance_);

  wind_direction_.Normalize();
  wind_gust_direction0_.Normalize();
  wind_gust_direction1_.Normalize();
  wind_gust_direction2_.Normalize();
  wind_gust_direction3_.Normalize();
  wind_gust_direction4_.Normalize();
  wind_gust_direction5_.Normalize();
  wind_gust_start0_ = common::Time(wind_gust_start0);
  wind_gust_end0_ = common::Time(wind_gust_start0 + wind_gust_duration0);
  wind_gust_start1_ = common::Time(wind_gust_start1);
  wind_gust_end1_ = common::Time(wind_gust_start1 + wind_gust_duration1);
  wind_gust_start2_ = common::Time(wind_gust_start2);
  wind_gust_end2_ = common::Time(wind_gust_start2 + wind_gust_duration2);
  wind_gust_start3_ = common::Time(wind_gust_start3);
  wind_gust_end3_ = common::Time(wind_gust_start3 + wind_gust_duration3);
  wind_gust_start4_ = common::Time(wind_gust_start4);
  wind_gust_end4_ = common::Time(wind_gust_start4 + wind_gust_duration4);
  wind_gust_start5_ = common::Time(wind_gust_start5);
  wind_gust_end5_ = common::Time(wind_gust_start5 + wind_gust_duration5);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_gust_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWindGustPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboWindGustPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current simulation time.
  common::Time now = world_->SimTime();

  // Calculate the wind force.
  double wind_strength = wind_force_mean_;
  ignition::math::Vector3d wind = wind_strength * wind_direction_;
  // Apply a force from the constant wind to the link.
  link_->AddForceAtRelativePosition(wind, xyz_offset_);

  ignition::math::Vector3d wind_gust(0, 0, 0);

    // Calculate the wind gust force.
  if (now >= wind_gust_start0_ && now < wind_gust_end0_) {
    wind_gust += wind_gust_force_mean0_*wind_gust_direction0_;
  }
  if (now >= wind_gust_start1_ && now < wind_gust_end1_) {
    wind_gust += wind_gust_force_mean1_*wind_gust_direction1_;
  }
  if (now >= wind_gust_start2_ && now < wind_gust_end2_) {
    wind_gust += wind_gust_force_mean2_*wind_gust_direction2_;
  }
  if (now >= wind_gust_start3_ && now < wind_gust_end3_) {
    wind_gust += wind_gust_force_mean3_*wind_gust_direction3_;
  }
  if (now >= wind_gust_start4_ && now < wind_gust_end4_) {
    wind_gust += wind_gust_force_mean4_*wind_gust_direction4_;
  }
  if (now >= wind_gust_start5_ && now < wind_gust_end5_) {
    wind_gust += wind_gust_force_mean5_*wind_gust_direction5_;
  }
    // Apply a force from the wind gust to the link.
  link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);

  wrench_stamped_msg_.mutable_header()->set_frame_id(frame_id_);
  wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_x(wind.X() +
                                                               wind_gust.X());
  wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_y(wind.Y() +
                                                               wind_gust.Y());
  wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_z(wind.Z() +
                                                               wind_gust.Z());

  // No torque due to wind, set x,y and z to 0.
  wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_x(0);
  wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_y(0);
  wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_z(0);

  wind_force_pub_->Publish(wrench_stamped_msg_);

  // Calculate the wind speed.
  double wind_speed = wind_speed_mean_;
  ignition::math::Vector3d wind_velocity = wind_speed * wind_direction_;

  wind_speed_msg_.mutable_header()->set_frame_id(frame_id_);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wind_speed_msg_.mutable_velocity()->set_x(wind_velocity.X());
  wind_speed_msg_.mutable_velocity()->set_y(wind_velocity.Y());
  wind_speed_msg_.mutable_velocity()->set_z(wind_velocity.Z());

  wind_speed_pub_->Publish(wind_speed_msg_);
}

void GazeboWindGustPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message.
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // ========= WRENCH STAMPED MSG SETUP ========= //
  // ============================================ //
  wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
      "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========== WIND SPEED MSG SETUP ============ //
  // ============================================ //
  wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs_rotors::WindSpeed>(
      "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindGustPlugin);

}  // namespace gazebo
