#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        //   // Just output a message for now
        //   std::cerr << "\nThe velodyne plugin is attach to model[" <<
        //     _model->GetName() << "]\n";
        // safety check
        if( _model-> GetJointCount() == 0) {
            std::cerr << "Invalid joint count, Velodyne plugin not loaded \n";
            return;
        }

        // Store the model pointer for convenience
        this->model = _model;
        
        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint = _model->GetJoints()[0]; // getting only the first joint.
    
        // Apply the P-controller, with a gain of 0.1
        this->pid = common::PID(0.1, 0, 0);
        
        // Apply the P-controller to the joint.
        this->model->GetJointController()->SetVelocityPID(
            this->joint->GetScopedName(), this->pid
        );

        // Set the joint's target velocity. This target velocity is just for
        // demonstration purposes. **NA now, EDITED**
        // Setting up the velocity set service.
        double velocity = 0;

        // Check if the velocity element exists, and then read the value.
        // This service's objects are contained by the sdf file, in this case
        // the .world file. i.e. we are literally checking whether we specified
        // the velocity tag under this plugin or not. 
        // check for <velocity>[value]</velocity> in ../velodyne.world
        if(_sdf->HasElement("velocity"))
            velocity = _sdf->Get<double>("velocity"); // getting the value contained in the tag and casting to double

        this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), velocity
        ); 
        
        // Create the node
        this->node = transport::NodePtr(new transport::Node());

        #if GAZEBO_MAJOR_VERSION < 8 //not really needed, but just in case
        this->node->Init(this->model->GetWorld()->GetName());
        #else
        this->node->Init(this->model->GetWorld()->Name());
        #endif

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

        // Subscribe to this topic, and register a callback
        this->sub = this->node->Subscribe(topicName,
        &VelodynePlugin::OnMsg, this);

    }
    // Now; just like this SetVelocityTarget() function provided an API to set the 
    // velocity of the model's joints. We will create a velocity setting API which other
    // ROS programs can use when required.


    /// \brief A node used for transport.
    /// \param[in] _vel New target velocity.
    public: void SetVelocity(const double &_vel) {
        // Set the joint's velocity.
        this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), _vel
        );
    }

    private:
        physics::ModelPtr model;
        physics::JointPtr joint;
        common::PID pid;
        transport::NodePtr node;
        transport::SubscriberPtr sub;
    
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg) {
        this->SetVelocity(_msg->x());
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif