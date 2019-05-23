#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "planar_bot_hw.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planar_bot_hw");
  
  planar_bot_hw::PlanarBotHW robot;
  controller_manager::ControllerManager cm(&robot);
  
  ros::Duration period(0.001);
  
  ros::Rate loop_rate(1000);
  unsigned int loop_counter = 0;

  while (ros::ok()) {
    ros::Time t(loop_counter*period.toSec());
    cm.update(ros::Time::now(), period);
    
    ros::spinOnce();
    loop_rate.sleep();
    loop_counter++;
  }
}
