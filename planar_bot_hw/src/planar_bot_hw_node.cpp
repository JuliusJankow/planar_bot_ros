#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <vector>

#include "planar_bot_hw.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planar_bot_hw");
  
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  
  std::vector<double> q_init(4);
  //q_init[0] = 2.0;
  //q_init[1] = 1.5;
  q_init[0] = 1.7;
  q_init[1] = -0.5;
  q_init[2] = -0.5;
  q_init[3] = 0.3;
  
  planar_bot_hw::PlanarBotHW robot(q_init);
  controller_manager::ControllerManager cm(&robot);
  
  double t0 = ros::Time::now().toSec();
  
  ros::Duration period(0.02);
  
  ros::Rate loop_rate(period);
  unsigned int loop_counter = 0;

  while (ros::ok()) {
    ros::Time t(t0 + loop_counter*period.toSec());
    
    cm.update(t, period);
    robot.update();

    if(!loop_rate.sleep()) {
      ROS_DEBUG("Time exceeded");
    }
    loop_counter++;
  }
}
