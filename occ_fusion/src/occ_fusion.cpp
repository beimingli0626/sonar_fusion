#include <occ_fusion/occ_fusion.h>


OccFusion::OccFusion(ros::NodeHandle &nh) 
{
  node_ = nh;

  node_.param("occ_map/origin_x", origin_(0), -20.0);
  node_.param("occ_map/origin_y", origin_(1), -20.0);
  node_.param("occ_map/origin_z", origin_(2), 0.0);
  node_.param("occ_map/map_size_x", map_size_(0), 40.0);
  node_.param("occ_map/map_size_y", map_size_(1), 40.0);
  node_.param("occ_map/map_size_z", map_size_(2), 5.0);
  node_.param("occ_map/resolution", resolution_, 0.2);
  node_.param("occ_map/resolution", resolution_, 0.2);
  node_.param("occ_fusion/x_search_range", x_search_range_, 5.0);
  node_.param("occ_fusion/y_search_range", y_search_range_, 5.0);

  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_);

  occ_map_.reset(new OccMap);
  occ_map_->init(nh);
  sonar_occ_map_.reset(new SonarOccMap);
  sonar_occ_map_->init(nh);

  plane_pcl_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_fusion/plane_pcl", 1);
  cam_occ_update_timer_ = node_.createTimer(ros::Duration(0.2), &OccFusion::camOccMapUpdateCallback, this);
  detect_window_timer_ = node_.createTimer(ros::Duration(0.2), &OccFusion::detectWindowCallback, this);

  // Create the segmentation object
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(1000);
  seg_.setDistanceThreshold(0.01);
  // detect_window_server_ = node_.advertiseService("/occ_fusion/detect_window", &OccFusion::detectWindow, this);
}

OccFusion::~OccFusion() {}

void OccFusion::camOccMapUpdateCallback(const ros::TimerEvent& e)
{
  occ_map_->getOccupancyBinary(cam_occupancy_buffer_);
  sonar_occ_map_->updateCamOccupancy(cam_occupancy_buffer_);
}

void OccFusion::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  pos = origin_;
  for (int i = 0; i < 3; ++i)
    pos(i) += (id(i) + 0.5) * resolution_;
}

void OccFusion::detectWindowCallback(const ros::TimerEvent& e)
{
  Eigen::Vector3i sonar_center_idx = sonar_occ_map_->getSonarCenterIndex();
  if (sonar_center_idx(0) == -1 && sonar_center_idx(1) == -1 && sonar_center_idx(2) == -1) {
    ROS_INFO("[Occ Fusion] no valid sonar center idx");
    return;
  }
 
  /* build point cloud that composes of points lie inside the cube
  *  the cube is centered at the sonar center idx
  *  can change the size of the cube in launch file, default 5m * 5m * height
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ point; //colored point clouds also have RGB values

  int x_start = min(max(0, sonar_center_idx(0) - int(x_search_range_ / 2 / resolution_)), grid_size_(0));
  int y_start = min(max(0, sonar_center_idx(1) - int(y_search_range_ / 2 / resolution_)), grid_size_(1));
  int x_end = min(x_start + int(x_search_range_ / resolution_), grid_size_(0));
  int y_end = min(y_start + int(y_search_range_ / resolution_), grid_size_(1));
  for (int x = x_start; x < x_end; x++)
  {
    for (int y = y_start; y < y_end; y++)
    {
      for (int z = 2; z < grid_size_(2); z++) // start from 2 so that we ignore the ground plane
      {
        int idx_ctns = x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z;
        if (cam_occupancy_buffer_[idx_ctns] == 1) // if occupied
        {
          Eigen::Vector3i idx(x,y,z);
          Eigen::Vector3d pos;
          indexToPos(idx, pos);
          point.x = pos[0];
          point.y = pos[1];
          point.z = pos[2];
          cloud->points.push_back(point);
        }
      }
    }
  }
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  int num_points = cloud->points.size();
  if (num_points != 0) {  // make sure we have enough poitn for segmentation
    ROS_INFO_STREAM("[Occ Fusion] Run plane segmentation on " << num_points << " points");
    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coefficients);

    extract_.setInputCloud(cloud);
    extract_.setIndices(inliers);
    extract_.filter(*extracted_cloud);

    cloud->width = (int)cloud->points.size();
    cloud->height = 1;    //height=1 implies this is not an "ordered" point cloud
    // Convert the cloud to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*extracted_cloud, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "world";
    plane_pcl_pub_.publish(output);
  }
}

// bool OccFusion::detectWindow(occ_fusion_msgs::DetectWindow::Request &req, 
//                             occ_fusion_msgs::DetectWindow::Response &res)
// {
//   ROS_INFO("[Occ Fusion] Receive Detect Window service call");
//   res.res_str = "Detect window success";
//   return true;
// }