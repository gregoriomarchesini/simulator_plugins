// This code was developed using a template from the gazebo_ros_package for ros2 humble 
// See https://github.com/ros2-gbp/gazebo_ros_pkgs-release/blob/release/humble/gazebo_plugins/src/gazebo_ros_force.cpp for details 

/*
 * \brief  This plugin is an implementation of the Clohessy–Wiltshire relative acceleration model for a spacecraft orbiting orbiting "relative to" an orbiting structure.
 *         This model is valid for a spacecraft orbiting in close proximity to a a larger structure like the ISS. The model can be appropriately applied when the orbit of the 
 *         the structure is nearly circular and the spacecraft is orbiting in close proximity to the structure (less than 30 km).
 *   
 * \author  Gregorio Marchesini (gremar@kth.se)
 *
 * \date  April 2024
 */

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class CWRelAccModelPrivate;

/*
 * \brief Acceleration model according to the Clohessy-Wiltshire equations. The chaser spacecraft (astrobee) is assumed to be revolving around the target (ISS for example) with the target moving ina  circular orbit around a massive body.
 *        The target is assumed to be centerred at the world pose (0,0,0) and the world frame of reference is assumed to be the RSW frame of the target orbit. The following defitinion of RSW frame holds
 *        - R : Radial direction pointing from the center of the massive planet to the target (outward from the planet a.k.a zenith)
 *        - W : Direction of the angular momentum of the orbit for the target (perpendicular to the orbit plane)
 *        - S : resulting direction perpendicular to the other two directions
 *        NOTE : The S directiuon is aligned with the velocity vector ONLY if the orbit is perfectly circular (which is assumed in this model). 
 *               In case the orbit of the target is eccentric, this assumption is not valid anymore and can cause implementation errors if mis-considered
 * 
 */


/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so">
        <cog_link_name>base_link</cog_link_name>      // frame representing the center of mass of the body
        <total_mass>300</total_mass>                  // in [kg]
        <orbit_radius>6778</orbit_radius>             // in [km]        (ISS example) 
        <gravity_parameter>398600</gravity_parameter> // in [km^3/s^2]  (Earth Example) (find for more planets at https://en.wikipedia.org/wiki/Standard_gravitational_parameter)
    </plugin>
  \endcode
*/

class CWRelAccModel : public gazebo::ModelPlugin
{
public:
  /// Constructor
  CWRelAccModel();

  /// Destructor
  ~CWRelAccModel();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<CWRelAccModelPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_