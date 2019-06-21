### Planar Bot ROS

## A ros project for testing planning and control approaches with minimalistic kinematics and a planar workspace

# Software Dependencies
- ros
- ros_control
- rviz
- gazebo_ros (only neccessary if you want to test torque interface controller)

# Packages
- planar_bot_description: contains urdf files and a simple launch file for visualizing a urdf
- planar_bot_hw: "Hardware" class, that maps controller command to joint state
  in order to link position interface controller to joint_state_controller
- planar_bot_gazebo: contains launch files to start the gazebo backend with a ros interface
- planar_bot_control: custom controllers that implement several inverse kinematic resolution approaches on position and torque level
