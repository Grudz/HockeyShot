// Include guard to prevent multiple declarations
#pragma once

#include <ros/ros.h> // General ROS functions

// Sensor messages
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Image.h> 
#include <sensor_msgs/CameraInfo.h> 

// Dynamic reconfigure stuff
#include <dynamic_reconfigure/server.h>  
#include <hockey_shot/HockeyShotConfig.h>

// Processing
#include <avs_lecture_msgs/TrackedObjectArray.h>
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>
#include <pcl/common/common.h>  // copy/paste point cloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>

//#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseArray.h>  // Visualize vectors
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Namespace matches ROS package name
namespace hockey_shot{

  class HockeyShot
  {
  public:
  
    HockeyShot(ros::NodeHandle n, ros::NodeHandle pn);
        
  private:

    // Lidar functions
    void recvPointsRaw(const sensor_msgs::PointCloud2ConstPtr& msg);  
    void recvPointsColored(const sensor_msgs::PointCloud2ConstPtr& msg); 

    // Camera functions
    void recvCameraRGB(const sensor_msgs::ImageConstPtr& msg);  
    void recvCameraDepth(const sensor_msgs::ImageConstPtr& msg);

    //void recvTF(const sensor_msgs::CameraInfoConstPtr& msg);  

    // These are unused currently
    void TimerCallback(const ros::TimerEvent& event); 
    void recvCameraRGBinfo(const sensor_msgs::CameraInfoConstPtr& msg);  
    void recvCameraDepthInfo(const sensor_msgs::CameraInfoConstPtr& msg);  
 
    // Dynmaic Reconfigure server
    void reconfig(HockeyShotConfig& config, uint32_t level);
    dynamic_reconfigure::Server<HockeyShotConfig> srv_;
    HockeyShotConfig cfg_;

    // Pubs, subs, timer
    ros::Timer timer;    
    ros::Subscriber sub_tf_;

    ros::Subscriber sub_camera_rgb_;
    ros::Subscriber sub_cam_depth_;
    ros::Subscriber sub_cam_rgb_info_;
    ros::Subscriber sub_cam_depth_info_;

    ros::Subscriber sub_points_raw_;
    ros::Subscriber sub_points_colored_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_passthrough_cloud_;
    ros::Publisher pub_bbox_;
    ros::Publisher pub_normals_;  // Normal vectors

    // KD search tree object for use by PCL functions
    pcl::search::Search<pcl::PointXYZRGB>::Ptr kd_tree_;

    // Output messages
    geometry_msgs::PoseArray normals_;

    // Publishing bounding box message
    avs_lecture_msgs::TrackedObjectArray bbox_array_;
    int bbox_id_;

    /*

    1) raw points - sensor_msgs/PointCloud2 - /camera/depth/points
    2) colored points - sensor_msgs/PointCloud2 - /camera/depth_registered/points
    3) camera RGB - sensor_msgs::image - /camera/rgb/image_rect_color
    4) camera debth - sensor_msgs::image - /camera/depth/image_rect
    5) rgb cam info - sensor_msgs/CameraInfo - /camera/rgb/camera_info
    6) depth cam info - sensor_msgs/CameraInfo - /camera/depth/camera_info
      
    */
  };

}


