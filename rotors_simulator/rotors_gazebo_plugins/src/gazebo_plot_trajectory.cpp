#include <rotors_gazebo_plugins/gazebo_plot_trajectory.hpp>

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    SomeVisualPlugin::SomeVisualPlugin():
      line(NULL)
    {

    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    SomeVisualPlugin::~SomeVisualPlugin()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void SomeVisualPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      this->visual_ = _parent;

      this->visual_namespace_ = "visual/";

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }

      std::cout << "VISUAL_PLUGIN: Plugin loaded" << std::endl;

      previous_point.x = 0;
      previous_point.y = 0;
      previous_point.z = 0;
      counter_ = 0;

      this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
      this->pose_sub_ = this->rosnode_->subscribe("/drone6/EstimatedPose_droneGMR_wrt_GFF", 1000, &SomeVisualPlugin::VisualizeTrajectory, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&SomeVisualPlugin::UpdateChild, this));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void SomeVisualPlugin::UpdateChild()
    {
      ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void SomeVisualPlugin::VisualizeTrajectory(const droneMsgsROS::dronePose::ConstPtr pose_msg)
    {
      if ((previous_point.x == 0) && (previous_point.y == 0) && (previous_point.z == 0)){
        previous_point.x = pose_msg->x;
        previous_point.y = pose_msg->y;
        previous_point.z = pose_msg->z - OFFSET;

        return;
      }

      counter_++;

      if (!(counter_ % COUNTER  == 0))
        return;

      this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

      std::cout << "Previous pose: " << previous_point.x << "   " << previous_point.y << "   " << previous_point.z << "   " << std::endl;
      std::cout << "Current pose: " << pose_msg->x << "   " << pose_msg->y << "   " << pose_msg->z - OFFSET << "   " << std::endl;

//      //TODO: Get the current link position
//      link_pose = CurrentLinkPose();
//      //TODO: Get the current end position
//      endpoint = CalculateEndpointOfForceVector(link_pose, force_msg);

      this->line->Clear();


//      // Add two points to a connecting line strip from link_pose to endpoint
      this->line->AddPoint(
        ignition::math::Vector3d(
          previous_point.x,
          previous_point.y,
          previous_point.z
          )
        );

      this->line->AddPoint(
        ignition::math::Vector3d(
          pose_msg->x,
          pose_msg->y,
          pose_msg->z - OFFSET
          )
        );

      previous_point.x = pose_msg->x;
      previous_point.y = pose_msg->y;
      previous_point.z = pose_msg->z - OFFSET;

//      this->line->AddPoint(math::Vector3(endpoint.x, endpoint.y, endpoint.z));
//      // set the Material of the line, in this case to purple
      this->line->setMaterial("Gazebo/Yellow");
      this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(SomeVisualPlugin)
  }
}
