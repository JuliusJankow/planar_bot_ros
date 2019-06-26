#include <planar_bot_planning/prm.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "planar_bot_prm");
  
  planar_bot_planning::PRM planner;
  
  ros::Rate rate(30);
  
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
