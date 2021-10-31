// Create fun gazebo city

// Header file for the class
#include "hockey_shot.h"

// Namespace matches ROS package name
namespace hockey_shot {

// Constructor with global and private node handle arguments
HockeyShot::HockeyShot(ros::NodeHandle n, ros::NodeHandle pn) : kd_tree_(new pcl::search::KdTree<pcl::PointXYZRGB>)
{

  // Timer
  timer = n.createTimer(ros::Duration(0.01), &HockeyShot::TimerCallback, this);

  // Reconfig
  srv_.setCallback(boost::bind(&HockeyShot::reconfig, this, _1, _2));

  // Camera stuff
  sub_camera_rgb_ = n.subscribe("/camera/rgb/image_rect_color", 1, &HockeyShot::recvCameraRGB, this);  
  sub_cam_depth_ = n.subscribe("/camera/depth/image_rect", 1, &HockeyShot::recvCameraDepth, this);  
  sub_cam_rgb_info_ = n.subscribe("/camera/rgb/camera_info", 1, &HockeyShot::recvCameraRGBinfo, this);  
  sub_cam_depth_info_ = n.subscribe("/camera/depth/camera_info", 1, &HockeyShot::recvCameraDepthInfo, this);  

  // Lidar stuff
  sub_points_raw_ = n.subscribe("/camera/depth/points", 1, &HockeyShot::recvPointsRaw, this);  
  sub_points_colored_ = n.subscribe("/camera/depth_registered/points", 1, &HockeyShot::recvPointsColored, this);  
  pub_cloud_= n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  pub_passthrough_cloud_= n.advertise<sensor_msgs::PointCloud2>("passthrough_cloud", 1);
  pub_bbox_= n.advertise<avs_lecture_msgs::TrackedObjectArray>("bounding_boxes", 1);

}  

  void HockeyShot::recvPointsColored(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    ROS_FATAL("Color Cloud");   

    // Create pointer to PCL type variable
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::fromROSMsg(*msg, *cloud_in); 

    //ROS_WARN("Cloud in frame id = %s", cloud_in->header.frame_id.c_str());
    ROS_WARN("Cloud_in size= %d", (int)cloud_in->size());

    // Downsample cloud
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_downsample;
    uniform_downsample.setInputCloud(cloud_in);
    uniform_downsample.setRadiusSearch(cfg_.uniform_thresh);
    uniform_downsample.filter(*cloud_in);

    ROS_WARN("Cloud_in size= %d", (int)cloud_in->size());

    // Start filtering
    pcl::IndicesPtr roi_indices(new std::vector <int>); // Will store subset of original array. ROI = region of interest
    pcl::PassThrough<pcl::PointXYZRGB> passthrough_filter; // Instantiate filter. Will find x, y, z points that will satisfy ROI

    // Put pointer to input cloud in passthrough filter
    passthrough_filter.setInputCloud(cloud_in);

    // Index is relative to the Lidar frame
    // Extract X points
    passthrough_filter.setFilterFieldName("x"); // 6 fields, hence PointXYZI
    passthrough_filter.setFilterLimits(cfg_.x_min, cfg_.x_max);
    passthrough_filter.filter(*roi_indices);    // Referes to input cloud

    // Extract Y points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("y"); 
    passthrough_filter.setFilterLimits(cfg_.y_min, cfg_.y_max);
    passthrough_filter.filter(*roi_indices);  

    // Extract Y points
    passthrough_filter.setIndices(roi_indices);
    passthrough_filter.setFilterFieldName("z"); 
    passthrough_filter.setFilterLimits(cfg_.z_min, cfg_.z_max);
    passthrough_filter.filter(*passthrough_cloud);  

    // Copy filtered data into a ROS message
    sensor_msgs::PointCloud2 pass_cloud;
    pcl::toROSMsg(*passthrough_cloud, pass_cloud);

    pass_cloud.header = msg->header;

    // Publish Passthrough filter PointCloud
    pub_passthrough_cloud_.publish(pass_cloud);
 
    ROS_WARN("Pass cloud size= %d", (int)passthrough_cloud->size());

    // Euclidean clustering 
    /*
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

    ec.setClusterTolerance(cfg_.cluster_tol);
    ec.setMinClusterSize(cfg_.min_cluster_size); 
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    kd_tree_->setInputCloud(passthrough_cloud);
    ec.setSearchMethod(kd_tree_);
    ec.setInputCloud(passthrough_cloud);
    ec.extract(cluster_indices);
    
    // Use indices arrays to separate point cloud into individual clouds for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_clouds;
    for (auto indices : cluster_indices) 
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::copyPointCloud(*passthrough_cloud, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster);
    }   
    // Use max and min of each cluster to create bbox
    pcl::PointXYZRGB min, max;  

    // Copy header from passthrough cloud and clear array
    bbox_array_.header = pcl_conversions::fromPCL(passthrough_cloud->header); // Nice way to get entire header easily
    bbox_array_.objects.clear();
    bbox_id_ = 0;

    // Loop through clusters and box up
    for (auto& cluster : cluster_clouds) 
    {  
      
      // Applying the min/max function
      pcl::getMinMax3D(*cluster, min, max);  // Get min/max 3D

      // Create bbox message, fill in fields, push it into bbox array
      avs_lecture_msgs::TrackedObject bbox;  // rosmsg show TrackedObjectArray
      
      bbox.header = bbox_array_.header;
      bbox.spawn_time.ros::Time::now(); // Spawn it right now
      bbox.id = bbox_id_++;
      bbox.bounding_box_scale.x = max.x - min.x;
      bbox.bounding_box_scale.y = max.y - min.y;
      bbox.bounding_box_scale.z = max.z - min.z;
      bbox.pose.position.x = (max.x + min.x) / 2; 
      bbox.pose.position.y = (max.y + min.y) / 2; 
      bbox.pose.position.z = (max.z + min.z) / 2; 
      bbox.pose.orientation.w = 1.0;

      bbox_array_.objects.push_back(bbox);
            
    } 
    
    pub_bbox_.publish(bbox_array_);
    */
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud (*passthrough_cloud, *indices);
    
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(passthrough_cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(kd_tree_);
    reg.setDistanceThreshold(cfg_.distance_thresh);
    reg.setPointColorThreshold(cfg_.pcolor_thresh);
    reg.setRegionColorThreshold(cfg_.rcolor_thresh);
    reg.setMinClusterSize(cfg_.min_cluster);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
   
    cloud_out = reg.getColoredCloud();

    // Copy filtered data into a ROS message
    sensor_msgs::PointCloud2 filtered_cloud;
    pcl::toROSMsg(*cloud_out, filtered_cloud);

    filtered_cloud.header = msg->header;

    // Publish Passthrough filter PointCloud
    pub_cloud_.publish(filtered_cloud);

  }

  void HockeyShot::recvPointsRaw(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    ROS_FATAL("Raw Cloud");
 
  }
  
  void HockeyShot::recvCameraRGB(const sensor_msgs::ImageConstPtr& msg)
  {


  }

  void HockeyShot::recvCameraDepth(const sensor_msgs::ImageConstPtr& msg)
  {


  }

  void HockeyShot::TimerCallback(const ros::TimerEvent& event)
  {


  } 

  void HockeyShot::reconfig(HockeyShotConfig& config, uint32_t level)
  {
    ROS_FATAL("Reconfig");
    cfg_ = config;

  }

  void HockeyShot::recvCameraRGBinfo(const sensor_msgs::CameraInfoConstPtr& msg)
  {


  }

  void HockeyShot::recvCameraDepthInfo(const sensor_msgs::CameraInfoConstPtr& msg)
  {


  }  
}
