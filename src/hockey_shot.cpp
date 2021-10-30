// Create fun gazebo city

// Header file for the class
#include "hockey_shot.h"

// Namespace matches ROS package name
namespace hockey_shot {

// Constructor with global and private node handle arguments
HockeyShot::HockeyShot(ros::NodeHandle n, ros::NodeHandle pn)
{

  timer = n.createTimer(ros::Duration(0.01), &HockeyShot::TimerCallback, this);


  sub_camera_rgb_ = n.subscribe("/camera/rgb/image_rect_color", 1, &HockeyShot::recvCameraRGB, this);  
  sub_cam_depth_ = n.subscribe("/camera/depth/image_rect", 1, &HockeyShot::recvCameraDepth, this);  
  sub_cam_rgb_info_ = n.subscribe("/camera/rgb/camera_info", 1, &HockeyShot::recvCameraDepth, this);  
  sub_cam_depth_info_ = n.subscribe("/camera/depth/camera_info", 1, &HockeyShot::recvCameraDepth, this);  

  sub_points_raw_ = n.subscribe("/camera/depth/points", 1, &HockeyShot::recvPointsRaw, this);  
  sub_points_colored_ = n.subscribe("/camera/depth_registered/points", 1, &HockeyShot::recvPointsColored, this);  

}  

  void HockeyShot::recvPointsColored(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {


  }

  void HockeyShot::recvPointsRaw(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {


  }
  
  void HockeyShot::recvCameraRGB(const sensor_msgs::Image::ConstPtr& msg)
  {


  }

  void HockeyShot::recvCameraDepth(const sensor_msgs::Image::ConstPtr& msg)
  {


  }

  void HockeyShot::TimerCallback(const ros::TimerEvent& event)
  {


  } 

  void HockeyShot::recvCameraRGBinfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {

    // Spitting out some error, but needed to get camera topic in bag

  }

  void HockeyShot::recvCameraDepthInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {

    // Spitting out some error, but needed to get camera topic in bag

  }  
}
