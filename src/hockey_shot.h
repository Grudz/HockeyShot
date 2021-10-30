// Include guard to prevent multiple declarations
#pragma once

#include <ros/ros.h> // General ROS functions

// Sensor messages
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Image.h> 
#include <sensor_msgs/CameraInfo.h> 

// Namespace matches ROS package name
namespace hockey_shot{

  class HockeyShot
  {
  public:
  
    HockeyShot(ros::NodeHandle n, ros::NodeHandle pn);
        
  private:

    ros::Timer timer;          
    ros::Subscriber sub_camera_rgb_;
    ros::Subscriber sub_cam_depth_;
    ros::Subscriber sub_cam_rgb_info_;
    ros::Subscriber sub_cam_depth_info_;
    ros::Subscriber sub_points_raw_;
    ros::Subscriber sub_points_colored_;
  
    /*

    1) raw points - sensor_msgs/PointCloud2 - /camera/depth/points
    2) colored points - sensor_msgs/PointCloud2 - /camera/depth_registered/points
    3) camera RGB - sensor_msgs::image - /camera/rgb/image_rect_color
    4) camera debth - sensor_msgs::image - /camera/depth/image_rect

    5) rgb cam info - sensor_msgs/CameraInfo - /camera/rgb/camera_info
    6) depth cam info - sensor_msgs/CameraInfo - /camera/depth/camera_info
      
    */

    void TimerCallback(const ros::TimerEvent& event); 
    void recvCameraRGB(const sensor_msgs::Image::ConstPtr& msg);  
    void recvCameraDepth(const sensor_msgs::Image::ConstPtr& msg);  
    void recvCameraRGBinfo(const sensor_msgs::CameraInfo::ConstPtr& msg);  
    void recvCameraDepthInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);  
    void recvPointsRaw(const sensor_msgs::PointCloud2::ConstPtr& msg);  
    void recvPointsColored(const sensor_msgs::PointCloud2::ConstPtr& msg);  
  
  };

}


