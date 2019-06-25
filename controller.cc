#ifndef _CONTROLLER_HH_
#define _CONTROLLER_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/dart/DARTPhysics.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/common/common.hh>
#include <functional>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"



namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class Controller : public ModelPlugin
  {
    /// \brief Constructor
    public: Controller() : ModelPlugin(){} 

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {    

        this->model = _model; 

      
        this->link = this->model->GetLink(
      _sdf->GetElement("link")->Get<std::string>());

	this->joint = this->model->GetJoint(
       _sdf->GetElement("joint")->Get<std::string>());

        this->pid = common::PID(0.1, 0, 0);
      
        this->model->GetJointController()->SetPositionPID(this->joint->GetScopedName(), this->pid);



      
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&Controller::OnUpdate, this));
      
     
      
    // Initialize ros, if it has not already bee initialized.
if (!ros::isInitialized())
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
}

// Create our ROS node. This acts in a similar manner to
// the Gazebo node
this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

// Create a named topic, and subscribe to it.
ros::SubscribeOptions so =
  ros::SubscribeOptions::create<std_msgs::Float32>(
      "/irobot_hand_nick/vel_cmd",
      1,
      boost::bind(&Controller::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);
this->rosSub = this->rosNode->subscribe(so);

// Spin up the queue helper thread.
this->rosQueueThread =
  std::thread(std::bind(&Controller::QueueThread, this));


	
}


 
    public: void OnUpdate()
    { 
	 
      
      
      //this->link->SetAngularVel({0,0,100});
      
    }

   public: void SetVelocity(const double &_vel)
   {
      //Set the joint's target velocity.
     this->link->AddTorque({0,0,_vel});
   }

   public: void SetJointPosition(const double &_pos){
	this->model->GetJointController()->SetPositionTarget(this->joint->GetScopedName(), _pos);
}
   /// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
{
  this->SetJointPosition(_msg->data);
}

/// \brief ROS helper function that processes messages
private: void QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
    
   

  
    
      
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::JointPtr joint;
   // private: physics::JointPtr joint; 
    private: common::PID pid;
    private: event::ConnectionPtr updateConnection;
   /// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;
   
  
      
    
  };


  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(Controller)
}
#endif
