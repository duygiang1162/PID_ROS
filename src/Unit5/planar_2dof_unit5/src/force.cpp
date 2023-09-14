#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo {
class AModelPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;
    this->link_name = "root";
    if (_sdf->HasElement("linkname")) {
      this->link_name = _sdf->Get<std::string>("linkname");
    }

    this->link1 = this->model->GetLink(this->link_name);
    if (!this->link1) {
      gzerr << "Root link are not found. (link_name is " << this->link_name
            << ")" << std::endl;
      return;
    }
    // Listen to the update event. This event is broadcast every
    // simulation iteration.

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AModelPlugin::OnUpdate, this));
  }

  // Called by the world update start event
public:
  void OnUpdate() {
    // ROS_INFO("Help me Obi-Wan Kenobi, you're my only hope");
    // this->link1->AddRelativeForce({5, 5, 0});
    this->link1->AddForceAtRelativePosition({5000, 5000, 0}, {0, 0, 0});
  }

  // Pointer to the model
private:
  physics::LinkPtr link1;
  physics::ModelPtr model;
  std::string link_name;

private:
  // int counter;
  // double linear_vel;
  // int iterations;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AModelPlugin)
} // namespace gazebo