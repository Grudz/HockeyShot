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
  pub_passthrough_cloud_= n.advertise<sensor_msgs::PointCloud2>("passthrough_cloud", 1);
  pub_color_cloud_ = n.advertise<sensor_msgs::PointCloud2>("color_cloud", 1);
  pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);

  cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE); // Setup window to view camera processing

}  

  void HockeyShot::recvCameraRGB(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // Kinda like PCL library, use this TYPE_32FC1
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat bin_img;
    cv::imshow("RGB", raw_img);
    cv::waitKey(1);

  }

  void HockeyShot::recvCameraDepth(const sensor_msgs::ImageConstPtr& msg)
  {

    // Convert ROS image message into an OpenCV Mat
    /*
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);  // Kinda like PCL library, use this TYPE_32FC1
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat bin_img;
    cv::imshow("Depth", raw_img);
    cv::waitKey(1);
    */

  }

  void HockeyShot::recvPointsColored(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    // Create pointer to PCL type variable
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filter_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::fromROSMsg(*msg, *cloud_in); 

    //ROS_WARN("Cloud in frame id = %s", cloud_in->header.frame_id.c_str());

    // Downsample cloud
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_downsample;
    uniform_downsample.setInputCloud(cloud_in);
    uniform_downsample.setRadiusSearch(cfg_.uniform_thresh);
    uniform_downsample.filter(*cloud_in);

    //ROS_WARN("Cloud_in size after downsample = %d", (int)cloud_in->size());

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

    //ROS_WARN("passthrough_cloud size = %d", (int)passthrough_cloud->size());

    // Copy filtered data into a ROS message
    sensor_msgs::PointCloud2 passthrough;
    pcl::toROSMsg(*passthrough_cloud, passthrough);

    passthrough.header = msg->header;

    pub_passthrough_cloud_.publish(passthrough);

    pcl::PointXYZRGB point;
    
    // ROS_WARN("Frame id = %s", passthrough_cloud->header.frame_id.c_str());
    for(auto& point : passthrough_cloud->points)
    {
      
      if (((int)point.r < cfg_.red) && ((int)point.b < cfg_.blue) && ((int)point.g < cfg_.green) && (double)point.z < cfg_.point_distance) 
      {
        /*
        ROS_WARN("Frame id = %s", passthrough_cloud->header.frame_id.c_str());
        std::cout << "X Distance = " << (double)point.x << std::endl;
        std::cout << "Y Distance = " << (double)point.y << std::endl;
        std::cout << "Z Distance = " << (double)point.z << std::endl;
        std::cout << "Red = " << (int)point.r << std::endl;
        std::cout << "Green = " << (int)point.g << std::endl;
        std::cout << "Blue = " << (int)point.b << std::endl;
        */
        color_filter_cloud->push_back(point);
      }
    }

    // Copy filtered data into a ROS message
    sensor_msgs::PointCloud2 color_cloud;
    pcl::toROSMsg(*color_filter_cloud, color_cloud);

    color_cloud.header = msg->header;

    // Publish Passthrough filter PointCloud
    pub_color_cloud_.publish(color_cloud);
 
    //ROS_WARN("color_cloud size= %d", (int)color_filter_cloud->size());

    // Compute normal vectors for the incoming point cloud, this is computational, so filter first
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);  // Create new cloud but type is normal not XYZ
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;  // Input PC XYZ, output Normal

    kd_tree_->setInputCloud(passthrough_cloud);  // Sort filtered cloud with KD tree to correspond with 3D position (sorts array)
    normal_estimator.setSearchMethod(kd_tree_);  // Use this ^^
    normal_estimator.setInputCloud(passthrough_cloud);  // Input cloud to process (filtered from previous stage)
    normal_estimator.setKSearch(cfg_.num_normal_neighbors);  // For each point, search for this many nearby ones. It fits them to plane equations then uses cross product to find normal
    normal_estimator.compute(*cloud_normals);  // (Cross product for normals)

    // TODO: Filter out normals here? Process them?
    pcl::PointIndices normals;  

    for (int i = 0; i < cloud_normals->points.size(); i++) 
    {
      double x = cloud_normals->points.at(i).normal_x;
      double y = cloud_normals->points.at(i).normal_y;
      double z = cloud_normals->points.at(i).normal_z;


      double angle_x = acos(abs(x)) * 180 / M_PI; // Z angle converted to radians
      double angle_y = acos(abs(y)) * 180 / M_PI; 
      double angle_z = acos(abs(z)) * 180 / M_PI; 

      if ((angle_x > cfg_.x_angle) || (angle_y > cfg_.y_angle) || (angle_z > 180)) 
      {
        //std::cout << "x angle = " << angle_x << "\n";
        //std::cout << "y angle = " << angle_y << "\n";
        //std::cout << "z angle = " << angle_z << "\n";

        continue;
      }

      normals.indices.push_back(i); 

    }

    int num_vectors;
    num_vectors = (int)normals.indices.size();
    
    ROS_INFO("No shot detected");
    if (num_vectors > cfg_.shot_indicator)
    {
      ROS_FATAL("SHOT");
      //ROS_INFO("Number of vectors = %d", num_vectors);
    }

    // Copy non-vertical normals into a separate cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  // Create another PC
    pcl::copyPointCloud(*passthrough_cloud, normals, *normal_cloud);  // (filtered output, non_vertical PC, stored in no ground cloud)

    // Populate PoseArray message to visualize normals
    normals_.header = pcl_conversions::fromPCL(passthrough_cloud->header);
    normals_.poses.clear();

    for (int i = 0; i < normals.indices.size(); i++)
    {

      geometry_msgs::Pose p;
      p.position.x = passthrough_cloud->points[normals.indices[i]].x;
      p.position.y = passthrough_cloud->points[normals.indices[i]].y;
      p.position.z = passthrough_cloud->points[normals.indices[i]].z;     

      double nx = cloud_normals->points[normals.indices[i]].normal_x;
      double ny = cloud_normals->points[normals.indices[i]].normal_y;
      double nz = cloud_normals->points[normals.indices[i]].normal_z;

      
      // Construct rotation matrix to align frame transform with the normal vector
      tf2::Matrix3x3 rot_mat;
      rot_mat[0] = tf2::Vector3(nx, ny, nz);

      if (std::abs(nz) < 0.9) 
      {
        // Vector is not close to vertical, use x and y components to create orthogonal vector
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
      } 
      else // Most points fall into this
      {
        // Vector is close to vertical, use y and z components to make orthogonal vector
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
      }

      // Normalize the generated orthogonal vector, not necessarily unit length so force 1 magnitude
      rot_mat[1].normalize();

      // Cross product produces the third basis vector (column) of the rotation matrix - creates perpindicular vector
      rot_mat[2] = rot_mat[0].cross(rot_mat[1]);

      // Extract equivalent quaternion representation for the transform
      // rot_mat.transpose() is used because the basis vectors should be loaded
      // into the columns of the matrix, but the indexing in the above commands set the rows
      // of the matrix instead of the columns.
      tf2::Quaternion q;
      rot_mat.transpose().getRotation(q);

      // Fill orientation of pose structure
      tf2::convert(q, p.orientation);
      normals_.poses.push_back(p);
      
    }

    // Publish normal vectors
    pub_normals_.publish(normals_);

  }

  void HockeyShot::recvPointsRaw(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

 
  }
  

  void HockeyShot::TimerCallback(const ros::TimerEvent& event)
  {


  } 

  void HockeyShot::reconfig(HockeyShotConfig& config, uint32_t level)
  {
    
    cfg_ = config;

  }

  void HockeyShot::recvCameraRGBinfo(const sensor_msgs::CameraInfoConstPtr& msg)
  {


  }

  void HockeyShot::recvCameraDepthInfo(const sensor_msgs::CameraInfoConstPtr& msg)
  {


  }  
}
