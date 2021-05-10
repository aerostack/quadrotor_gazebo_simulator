
#include <gazebo/physics/physics.hh>
#include "rotors_gazebo_plugins/gazebo_random_position_plugin.h"

#include "ConnectGazeboToRosTopic.pb.h"


namespace gazebo {

GazeboRandomPositionPlugin::~GazeboRandomPositionPlugin() {
    //  Trying to adapt to Gazebo9
    //  event::Events::ConnectWorldUpdateEnd(update_connection_);
}




/////////////////////////////////////////////////
void GazeboRandomPositionPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->model = _model;

    if (_sdf->HasElement("limitX")) {
      limit_x_ = _sdf->Get<double>("limitX");
    }
    if (_sdf->HasElement("limitY")) {
      limit_y_ = _sdf->Get<double>("limitY");
    }

    if (_sdf->HasElement("limitZ")) {
      limit_z_ = _sdf->Get<double>("limitZ");
    }

    if (_sdf->HasElement("activate_roll")) {
      roll_ = _sdf->Get<bool>("activate_roll");
    }

}
/////////////////////////////////////////////
void GazeboRandomPositionPlugin::Init()
{
    ignition::math::Pose3d pose = this->model->WorldPose();
    float y0 = pose.Pos().Y();

    // Get the desired pose, here giving a random offset
    if (roll_){
        randroll =  ignition::math::Rand::DblUniform(0, 1);
        if (randroll < 0.25){
//            pose += ignition::math::Pose3f(0, 0, 0, 0, 0, 0);
        }
        else{
            if (randroll < 0.5){
                pose = this->model->WorldPose();
                pose += ignition::math::Pose3d(0, 0, 0, 1.57, 0, 0);
                this->model->SetWorldPose(pose);
                pose = this->model->WorldPose();
                pose += ignition::math::Pose3d(0, y0, -y0+2.1, 0, 0, 0);
                this->model->SetWorldPose(pose);
            }
            else{
                if (randroll < 0.75){
                    pose = this->model->WorldPose();
                    pose += ignition::math::Pose3d(0, 0, 0, 3.14, 0, 0);
                    this->model->SetWorldPose(pose);
                    pose = this->model->WorldPose();
                    pose += ignition::math::Pose3d(0, 2*y0-2.1, 2.1, 0, 0, 0);
                    this->model->SetWorldPose(pose);
                }
                else{
                    pose = this->model->WorldPose();
                    pose += ignition::math::Pose3d(0, 0, 0, -1.57, 0, 0);
                    this->model->SetWorldPose(pose);
                    pose = this->model->WorldPose();
                    pose += ignition::math::Pose3d(0, y0-2.1, y0, 0, 0, 0);
                    this->model->SetWorldPose(pose);
                }
            }
        }
    }
    if (limit_x_ == 0) x = 0;
    else x = ignition::math::Rand::DblUniform(-limit_x_, limit_x_);
    if (limit_y_ == 0) y = 0;
    else y = ignition::math::Rand::DblUniform(-limit_y_, limit_y_);
    if (limit_z_==0) z = 0;
    else z = ignition::math::Rand::DblUniform(-limit_z_, limit_z_);
    pose += ignition::math::Pose3d(-x, -y, z, 0, 0, 0);
    this->model->SetWorldPose(pose);
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRandomPositionPlugin)

}
