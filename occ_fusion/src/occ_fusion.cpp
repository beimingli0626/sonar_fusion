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
  node_.param("occ_fusion/vis_plane_pcl", vis_plane_pcl_, true);

  resolution_inv_ = 1 / resolution_;
  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_);
  window_marker_id_ = 0;

  occ_map_.reset(new OccMap);
  occ_map_->init(nh);
  sonar_occ_map_.reset(new SonarOccMap);
  sonar_occ_map_->init(nh);

  plane_pcl_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_fusion/plane_pcl", 1);
  window_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/occ_fusion/window_marker", 1);
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

void OccFusion::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
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
  
  /* use SACSegmentation to find largest planar components*/
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  int num_points = cloud->points.size();
  if (num_points != 0) {  // make sure we have enough points for segmentation
    ROS_INFO_STREAM("[Occ Fusion] Run plane segmentation on " << num_points << " points");

    // While 30% of the original cloud is still there
    int num_planes = 0;
    while (cloud->points.size () > 0.3 * num_points) {
      // Segment the largest planar component from the remaining cloud
      seg_.setInputCloud(cloud);
      seg_.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      num_planes++;
      // stop if have find at least one plane almost perpendicular to x-y plane
      if (std::abs(coefficients->values[2]) < 0.02) break;

      // if not, keep search the rest
      extract_.setIndices(inliers);
      extract_.setNegative(true);
      extract_.filter(*rest_cloud);
      cloud.swap(rest_cloud);
    }

    if (num_planes > 0 && std::abs(coefficients->values[2]) < 0.02) {
      ROS_INFO_STREAM("[Occ Fusion] Plane Coefficients " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3]);
      detectOpen(coefficients->values[0], coefficients->values[1], coefficients->values[3], sonar_center_idx);
    }

    /* publish plane pointcloud */
    if (vis_plane_pcl_) {
      extract_.setInputCloud(cloud);
      extract_.setIndices(inliers);
      extract_.filter(*extracted_cloud);
      extracted_cloud->width = (int)extracted_cloud->points.size();
      extracted_cloud->height = 1;    //height=1 implies this is not an "ordered" point cloud
      // Convert the cloud to ROS message
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*extracted_cloud, output);
      output.header.stamp = ros::Time::now();
      output.header.frame_id = "world";
      plane_pcl_pub_.publish(output);
    }
  }
}

void OccFusion::detectOpen(float coeff_x, float coeff_y, float coeff_const, Eigen::Vector3i &sonar_center_idx) 
{
  std::vector<int> distance;  // vertical distances
  std::vector<Eigen::Vector3i> upper_bound;
  std::vector<Eigen::Vector3i> lower_bound;
  int upper_z, lower_z;

  // detect vertical gaps line by line
  if (std::abs(coeff_x) > std::abs(coeff_y))  // iterate based on y
  {
    int y_start = min(max(0, sonar_center_idx(1) - int(y_search_range_ / 2 / resolution_)), grid_size_(1)); // idx
    int y_end = min(y_start + int(y_search_range_ / resolution_), grid_size_(1)); // idx
    for (int y = y_start; y < y_end; y++) {
      double y_world = origin_(1) + (y + 0.5) * resolution_;
      double x_world = (-coeff_const - coeff_y * y_world) / coeff_x;
      int x = floor((x_world - origin_(0)) * resolution_inv_);
      for (upper_z = sonar_center_idx(2); upper_z < grid_size_(2); upper_z++) { // search for upper bound
        if (cam_occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + upper_z] == 1) { // reach an occupied space
          break;
        }
      }
      for (lower_z = sonar_center_idx(2) - 1; lower_z >= 2; lower_z--) {  // search for lower bound
        if (cam_occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + lower_z] == 1) { // reach an occupied space
          break;
        }
      }
      distance.push_back(int(upper_z - lower_z));
      lower_bound.push_back(Eigen::Vector3i(x, y, lower_z));
      upper_bound.push_back(Eigen::Vector3i(x, y, upper_z));
      // ROS_INFO_STREAM("upper height: " << upper_z << " lower height: " << lower_z);
      // ROS_INFO_STREAM("x_world: " << x_world << " y_world: " << y_world << "x: " << x << " y: " << y);
    }
  }
  else // iterate based on x
  {
    int x_start = min(max(0, sonar_center_idx(0) - int(x_search_range_ / 2 / resolution_)), grid_size_(0));
    int x_end = min(x_start + int(x_search_range_ / resolution_), grid_size_(0));
    for (int x = x_start; x < x_end; x++) {
      double x_world = origin_(0) + (x + 0.5) * resolution_;
      double y_world = (-coeff_const - coeff_x * x_world) / coeff_y;
      int y = floor((y_world - origin_(1)) * resolution_inv_);
      for (upper_z = sonar_center_idx(2); upper_z < grid_size_(2); upper_z++) { // search for upper bound
        if (cam_occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + upper_z] == 1) { // reach an occupied space
          break;
        }
      }
      for (lower_z = sonar_center_idx(2) - 1; lower_z >= 2; lower_z--) {  // search for lower bound
        if (cam_occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + lower_z] == 1) { // reach an occupied space
          break;
        }
      }
      distance.push_back(upper_z - lower_z);
      lower_bound.push_back(Eigen::Vector3i(x, y, lower_z));
      upper_bound.push_back(Eigen::Vector3i(x, y, upper_z));
      // ROS_INFO_STREAM("upper height: " << upper_z << " lower height: " << lower_z);
      // ROS_INFO_STREAM("x_world: " << x_world << " y_world: " << y_world << "x: " << x << " y: " << y);
    }
  }

  // record window corner
  bool window_start = false;
  int start_idx;
  Eigen::Vector3d pos;
  vector<Eigen::Vector3d> corners_pos;
  for (int i = 0; i < distance.size(); i++) {
    if (!window_start && distance[i] >= 10 && distance[i] <= 50) {  // window should be between 0.5m and 2.5m in height
      ROS_INFO("[Occ Fusion] find the start of window");
      window_start = true;  // window start
      start_idx = i;
      indexToPos(lower_bound[i], pos); // lower bound
      corners_pos.push_back(pos);
      indexToPos(upper_bound[i], pos); // upper bound
      corners_pos.push_back(pos);
    }
    else if (window_start && (distance[i] < 10 || distance[i] > 50)) {
      window_start = false;
      if (i - start_idx < 10) { // the window should at least 0.5m in width
        corners_pos.clear();
      }
      else {
        indexToPos(lower_bound[i - 1], pos); // lower bound
        corners_pos.push_back(pos);
        indexToPos(upper_bound[i - 1], pos); // upper bound
        corners_pos.push_back(pos);
        break;  // find the end of window
      }
    }
  }

  if (corners_pos.size() == 4) {
    // publish marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "box";
    marker.id = ++window_marker_id_;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the position, orientation, and scale of the box
    marker.pose.position.x = (corners_pos[0](0) + corners_pos[2](0)) / 2;
    marker.pose.position.y = (corners_pos[0](1) + corners_pos[2](1)) / 2;
    marker.pose.position.z = (corners_pos[0](2) + corners_pos[1](2) + corners_pos[2](2) + corners_pos[3](2)) / 4;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = max(std::abs(corners_pos[0](0) - corners_pos[2](0)), 0.1);
    marker.scale.y = max(std::abs(corners_pos[0](1) - corners_pos[2](1)), 0.1);
    marker.scale.z = (corners_pos[1](2) - corners_pos[0](2) + corners_pos[3](2) - corners_pos[2](2)) / 2;

    // Set the color of the box
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.9;

    marker.lifetime = ros::Duration(1000);

    // Publish the Marker message
    window_marker_pub_.publish(marker);
  }
}

// bool OccFusion::detectWindow(occ_fusion_msgs::DetectWindow::Request &req, 
//                             occ_fusion_msgs::DetectWindow::Response &res)
// {
//   ROS_INFO("[Occ Fusion] Receive Detect Window service call");
//   res.res_str = "Detect window success";
//   return true;
// }