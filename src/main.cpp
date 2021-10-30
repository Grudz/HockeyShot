// ROS and node class header file
#include <ros/ros.h>
#include "hockey_shot.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "hockey_shot");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  hockey_shot::HockeyShot node(n, pn);

  // Multithreaded, 0 = many threads as needed
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  // Since multithreaded, do this
  ros::waitForShutdown(); // ros::spin();
}
