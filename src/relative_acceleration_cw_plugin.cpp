// This code was developed using a template from the gazebo_ros_package for ros2 humble 
// See https://github.com/ros2-gbp/gazebo_ros_pkgs-release/blob/release/humble/gazebo_plugins/src/gazebo_ros_force.cpp for details 

/*
 * \brief  Implements the Cloesshy Wiltshire relative acceleration model for an orbiting model around the ISS
 *
 * \author  Gregorio Marchesini (gremar@kth.com)
 *
 * \date  April 2024
 */

#include <gazebo/common/Events.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/util/system.hh>


#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "simulator_plugins/relative_acceleration_cw_plugin.hh"
#include "simulator_msgs/srv/compute_relative_initial_conditions.hpp"   

namespace gazebo_plugins
{
class CWRelAccModelPrivate
{
public:

  /// Callback to be called at every simulation iteration (Here we add the relative acceleration to the model)
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  void CreateIntialConditionsService(const std::shared_ptr<simulator_msgs::srv::ComputeRelativeInitialConditions::Request>  request,     // CHANGE
                                           std::shared_ptr<simulator_msgs::srv::ComputeRelativeInitialConditions::Response> response);

  
  
  
  
  
  
  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// this service will be called to compute the initial conditions of the vehicle base on the desired parameteric orbit
  rclcpp::Service<simulator_msgs::srv::ComputeRelativeInitialConditions>::SharedPtr orbit_params_to_state_service_; 
  
  /// this service will be called to set/reset the state of the vehicle
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr state_setter_client_;
  

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// name of the com link
  std::string com_link_name_;
  
  /// model name
  std::string model_name;
  
  /// A pointer to the center of mass frame (the force will be applied in the center of mass but we will use use the world coordinates)
  gazebo::physics::LinkPtr com_link_;

  /// model mass
  double mass_;

  /// orbit radius (r)
  double orbit_radius_;


  /// gravity parameter (mu)
  double gravity_parameter_;

  /// orbit angular rate sqrt(mu/r^3)
  double n_rate_;
  double n_rate_squared_;

  /// WrenchedStamped message publisher.
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_;


  /// Container for the wrench force that this plugin exerts on the body.
  geometry_msgs::msg::Wrench wrench_msg_;

};

// Public implementation constructor
CWRelAccModel::CWRelAccModel() : impl_(std::make_unique<CWRelAccModelPrivate>())
{
}


// Public implementation deconstructor 
CWRelAccModel::~CWRelAccModel()
{
}

void CWRelAccModel::Reset()
{
}


void CWRelAccModel::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->pub_      = impl_->ros_node_->create_publisher<geometry_msgs::msg::Wrench>("cw_relative_acceleration", 1);

  impl_->orbit_params_to_state_service_  =  impl_->ros_node_->create_service<simulator_msgs::srv::ComputeRelativeInitialConditions>(
                                                            "compute_initial_conditions", std::bind(&CWRelAccModelPrivate::CreateIntialConditionsService, impl_.get(), std::placeholders::_1, std::placeholders::_2));

  
  impl_->state_setter_client_            =   impl_->ros_node_->create_client<gazebo_msgs::srv::SetEntityState>("mocap_simulator/set_entity_state"); // this has to match the service name in the gazebo_ros package  


  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Loading Plugin...");
  
  impl_->model_ = model;

  impl_->world_ = impl_->model_->GetWorld();
  
 

  if (!sdf->HasElement("com_link_name")){
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "CWRelAccModel plugin missing <com_link_name> element");
  } 
  
  impl_->com_link_name_ = sdf->Get<std::string>("com_link_name","base_frame").first;
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Center of mass link: %s", impl_->com_link_name_.c_str());
 

  

  if (!sdf->HasElement("total_mass")){
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),  "CWRelAccModel plugin missing <total_mass> element");
  } 
  impl_->mass_ =  sdf->Get<double>("total_mass",1).first ;
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Total mass: %f kg", impl_->mass_);
  

  

  if (!sdf->HasElement("orbit_radius")){
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),"CWRelAccModel plugin missing <orbit_radius> element");
  } 
  impl_->orbit_radius_ =sdf->Get<double>("orbit_radius",1).first;
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Orbit radius: %f km", impl_->orbit_radius_);


  if (!sdf->HasElement("gravity_parameter")){
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),"CWRelAccModel plugin missing <gravity_parameter> element"); 
  }
  impl_->gravity_parameter_ = sdf->Get<double>("gravity_parameter",0).first;
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Gravity parameter: %f km^3/s^2", impl_->gravity_parameter_);

  // Find target link
  // get a list if links 
  impl_->model_->GetLinks();
  // print the list of links
  for (auto link : impl_->model_->GetLinks()){
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Link: %s", link->GetName().c_str());
  }

  if (impl_->model_->GetLink(impl_->com_link_name_) == nullptr){
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "CWRelAccModel plugin could not find link: [%s]", impl_->com_link_name_.c_str());
  }
  impl_->com_link_ = impl_->model_->GetLink(impl_->com_link_name_);

  // orbit rate computation
  impl_->n_rate_ = sqrt(impl_->gravity_parameter_/pow(impl_->orbit_radius_,3));
  impl_->n_rate_squared_ = impl_->n_rate_*impl_->n_rate_;
 
  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&CWRelAccModelPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  // Retrieve and log the model name
  impl_->model_name = impl_->model_->GetName();
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Attached model name: %s", impl_->model_name.c_str());

}



void CWRelAccModelPrivate::CreateIntialConditionsService(const std::shared_ptr<simulator_msgs::srv::ComputeRelativeInitialConditions::Request>  request,     // CHANGE
                                                               std::shared_ptr<simulator_msgs::srv::ComputeRelativeInitialConditions::Response> response)
{

  
  auto client_request_for_spawning  = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  



  double rho_r = request->phase_amplitude.rho_r;
  double rho_s = request->phase_amplitude.rho_s;
  double rho_w = request->phase_amplitude.rho_w;

  double alpha_r = request->phase_amplitude.alpha_r;
  double alpha_w = request->phase_amplitude.alpha_w;

  /// this response is automatically sent back from ros when the function has terminated

  
  response->entity_state.name="cubot"; //todo/ automate this
  response->entity_state.pose.position.x    = rho_r*std::sin(alpha_r);
  response->entity_state.pose.position.y    = rho_s + 2*rho_r*std::cos(alpha_r);
  response->entity_state.pose.position.z    = rho_w*std::sin(alpha_w);
  response->entity_state.pose.orientation.x =  0.00;
  response->entity_state.pose.orientation.y =  0.00;
  response->entity_state.pose.orientation.z =  0.00;
  response->entity_state.pose.orientation.w =  0.00;
  response->entity_state.twist.linear.x     =  n_rate_* rho_r*std::cos(alpha_r);
  response->entity_state.twist.linear.y     = -n_rate_* rho_r * 2*rho_r*std::sin(alpha_r);
  response->entity_state.twist.linear.z     =  n_rate_* rho_w*std::cos(alpha_w);
  response->entity_state.twist.angular.x    = 0.00;
  response->entity_state.twist.angular.y    = 0.00;
  response->entity_state.twist.angular.z    = 0.00;
  response->entity_state.reference_frame="world";



  client_request_for_spawning->state.name="cubot"; //todo/ automate this
  client_request_for_spawning->state.pose.position.x    = rho_r*std::sin(alpha_r);
  client_request_for_spawning->state.pose.position.y    = rho_s + 2*rho_r*std::cos(alpha_r);
  client_request_for_spawning->state.pose.position.z    = rho_w*std::sin(alpha_w);
  client_request_for_spawning->state.pose.orientation.x = 0.00;
  client_request_for_spawning->state.pose.orientation.y = 0.00;
  client_request_for_spawning->state.pose.orientation.z = 0.00;
  client_request_for_spawning->state.pose.orientation.w = 0.00;
  client_request_for_spawning->state.twist.linear.x     =  n_rate_* rho_r*std::cos(alpha_r);
  client_request_for_spawning->state.twist.linear.y     = -n_rate_* rho_r * 2*rho_r*std::sin(alpha_r);
  client_request_for_spawning->state.twist.linear.z     =  n_rate_* rho_w*std::cos(alpha_w);
  client_request_for_spawning->state.twist.angular.x    = 0.00;
  client_request_for_spawning->state.twist.angular.y    = 0.00;
  client_request_for_spawning->state.twist.angular.z    = 0.00;
  client_request_for_spawning->state.reference_frame="world";


  /// call service from within this service 
  state_setter_client_->async_send_request(client_request_for_spawning);



}




void CWRelAccModelPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
    // Adding relative acceleration according to linear CW model
    // The force is expressed in the world frame which is the RSW frame of 
    // the body around which the free flyer is orbiting around

    
    std::lock_guard<std::mutex> scoped_lock(lock_); // lock the model
    ignition::math::Pose3d pose = model_->WorldPose();
    ignition::math::Vector3d vel =  model_->WorldLinearVel();

    double force_x = this->mass_ * ( 2 * this-> n_rate_ * vel.Y() + 3 * this->n_rate_squared_ * pose.Pos().X()); // out ward radial direction from the earth to the target body (from earth to international space station for example)
    double force_y = this->mass_ * (-2 * this->n_rate_ * vel.X());  // along track direction 
    double force_z = this->mass_ * (-this->n_rate_squared_ * pose.Pos().Z()); // no force in the normal direction



    wrench_msg_.force.x = force_x;
    wrench_msg_.force.y = force_y;
    wrench_msg_.force.z = force_z;
    wrench_msg_.torque.x = 0;
    wrench_msg_.torque.y = 0;
    wrench_msg_.torque.z = 0;

    

    // Apply the force (applied at one time step in the simulation)
    this->com_link_->AddForce(ignition::math::Vector3d(force_x, force_y, force_z));
    
    /// publish the model force
    pub_->publish(wrench_msg_);

}


GZ_REGISTER_MODEL_PLUGIN(CWRelAccModel)
}  // namespace gazebo_plugins