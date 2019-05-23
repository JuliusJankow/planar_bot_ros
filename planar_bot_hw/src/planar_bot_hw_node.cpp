#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "planar_bot_hw.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planar_bot_hw");
  
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  
  planar_bot_hw::PlanarBotHW robot;
  controller_manager::ControllerManager cm(&robot);
  
  double t0 = ros::Time::now().toSec();
  
  ros::Duration period(0.001);
  
  ros::Rate loop_rate(1000);
  unsigned int loop_counter = 0;

  while (ros::ok()) {
    ros::Time t(t0 + loop_counter*period.toSec());
    
    cm.update(t, period);
    robot.update();

    loop_rate.sleep();
    loop_counter++;
  }
}
