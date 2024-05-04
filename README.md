[![SHILED1](https://img.shields.io/badge/SPACE-Robotics-orange.svg)](link_to_your_project) [![SHILED1](https://img.shields.io/badge/KTH-DHSG-green.svg)](link_to_your_project) [![SHILED1](https://img.shields.io/badge/ROS-Humble-blue.svg)](link_to_your_project) [![SHILED1](https://img.shields.io/badge/GAZEBO-Classic_v11-blue.svg)](link_to_your_project)

# Gazebo Plugins for ROS 2 Simulator

This folder contains custom Gazebo plugins developed for the International Space Station (ISS) simulation project using ROS 2 and Gazebo Classic 11.

## Plugins

The Gazebo plugins provided in this repository enhance the simulation environment for space robotics and inspection missions. They include:

- `CWRelAccModel`: Gazebo plugin implementing a Clohessy-Wiltshire relative acceleration model for orbital maneuvers.
- `ISSCollisionModel`: Gazebo plugin defining rough collision models for the ISS to optimize computational performance in Gazebo simulations.
- `TrajectoryDrawing`: Gazebo plugin for drawing trajectories and paths in RViz for visualization and analysis.

These plugins enable complex behaviors and interactions within the Gazebo simulation environment, facilitating realistic space mission simulations and research.

## Installation

Clone this repository folder into your Gazebo plugins directory and build it using `catkin_make` or `colcon`.

```bash
cd /path/to/your/gazebo_plugins_directory
git clone <repository_url>/simulator_plugins
```


# Developers
Gregorio Marchesini [gremar@kth.se](mailto:gremar@kth.se)

Pedro Roque [padr@kth.se](padr@kth.se)

