#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ANGULAR_VELOCITY_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ANGULAR_VELOCITY_PLUGIN_H

#include <string>
#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h>

#include "rotors_gazebo_plugins/common.h"



namespace gazebo
{

	// Default values
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

static constexpr double kDefaultspeed = 0.0;


  class GazeboAngularVelPlugin : public ModelPlugin
  {
		public:
      GazeboAngularVelPlugin()
					: ModelPlugin(),
					  namespace_(kDefaultNamespace),

            speed(kDefaultspeed),

					  frame_id_(kDefaultFrameId),
            link_name_(kDefaultLinkName),
            node_handle_(nullptr),
            pubs_and_subs_created_(false){}

    virtual ~GazeboAngularVelPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    private: void OnUpdate(const common::UpdateInfo & /*_info*/);


    private:

      std::string namespace_;
      double speed;
      std::string frame_id_;
      std::string link_name_;

      physics::ModelPtr model;
      physics::JointPtr joint;
      physics::LinkPtr link_;
      event::ConnectionPtr update_connection_;

      gazebo::transport::NodePtr node_handle_;
      bool pubs_and_subs_created_;



  };
} // namespace gazebo 

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_ANGULAR_VELOCITY_PLUGIN_H
