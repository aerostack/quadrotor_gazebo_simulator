#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_RANDOM_POSITION_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_RANDOM_POSITION_PLUGIN_H

#include <string>
#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Pose3.hh>

#include <mav_msgs_rotors/default_topics.h>

#include "rotors_gazebo_plugins/common.h"



namespace gazebo
{

	// Default values
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

static constexpr double kDefaultlimitX = 0.0;
static constexpr double kDefaultlimitY = 0.0;
static constexpr double kDefaultlimitZ = 0.0;
static constexpr bool kDefaultroll = 0;

  class GazeboRandomPositionPlugin : public ModelPlugin
  {
		public:
			GazeboRandomPositionPlugin()
					: ModelPlugin(),
					  namespace_(kDefaultNamespace),

					  limit_x_(kDefaultlimitX),
					  limit_y_(kDefaultlimitY),
					  limit_z_(kDefaultlimitZ),
						roll_(kDefaultroll),

					  frame_id_(kDefaultFrameId),
            link_name_(kDefaultLinkName),
            node_handle_(nullptr),
            pubs_and_subs_created_(false){}

    virtual ~GazeboRandomPositionPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();


    private:

      std::string namespace_;
      double limit_x_;
      double limit_y_;
      double limit_z_;
      bool roll_;
      std::string frame_id_;
      std::string link_name_;

      physics::ModelPtr model;
      physics::LinkPtr link_;
      event::ConnectionPtr update_connection_;

      gazebo::transport::NodePtr node_handle_;
      bool pubs_and_subs_created_;

      float randroll;
      float roll;
      float x;
      float y;
      float z;


  };
} // namespace gazebo 

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_RANDOM_POSITION_PLUGIN_H
