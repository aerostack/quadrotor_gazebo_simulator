
#include <gazebo/physics/physics.hh>
#include "rotors_gazebo_plugins/gazebo_angular_vel_plugin.h"

#include "ConnectGazeboToRosTopic.pb.h"


namespace gazebo {

GazeboAngularVelPlugin::~GazeboAngularVelPlugin() {
    //  Trying to adapt to Gazebo9
    //  event::Events::ConnectWorldUpdateEnd(update_connection_);
}




/////////////////////////////////////////////////
void GazeboAngularVelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, plugin not loaded\n";
      return;
    }

    this->model = _model;
    this->joint = _model->GetJoint("moving");


    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboAngularVelPlugin::OnUpdate, this, _1));

    if (_sdf->HasElement("speed")) {
      speed = _sdf->Get<double>("speed");
    }


}

/////////////////////////////////////////////
void GazeboAngularVelPlugin::OnUpdate(const common::UpdateInfo &)
{
     this->joint->SetVelocity(0, speed);
}

 //Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboAngularVelPlugin)

}
