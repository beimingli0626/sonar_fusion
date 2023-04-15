#include "sonar_occ_grid/sonar_occ_map.h"
#include "sonar_occ_grid/raycast.h"
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//for img debug
#include <opencv2/opencv.hpp>

void SonarOccMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range_(0));
  min_pos(1) = max(min_pos(1), min_range_(1));
  min_pos(2) = max(min_pos(2), min_range_(2));

  max_pos(0) = min(max_pos(0), max_range_(0));
  max_pos(1) = min(max_pos(1), max_range_(1));
  max_pos(2) = min(max_pos(2), max_range_(2));

  Eigen::Vector3i min_id, max_id;

  posToIndex(min_pos, min_id);
  posToIndex(max_pos - Eigen::Vector3d(resolution_ / 2, resolution_ / 2, resolution_ / 2), max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] = clamp_min_log_;
      }
}

// This is used to clear all the cached data in all buffers, for continuously building new maps
void SonarOccMap::resetAllBuffer() {
  have_odom_ = false;
  global_map_valid_ = false;
  local_map_valid_ = false;
  has_global_cloud_ = false;
  has_first_depth_ = false;
  buffer_reset_ = true;

  raycast_num_ = 0;
  proj_points_cnt_ = 0;
  unknown_flag_ = 0.01;
 
  fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), clamp_min_log_ - unknown_flag_);

  fill(cache_all_.begin(), cache_all_.end(), 0);
  fill(cache_hit_.begin(), cache_hit_.end(), 0);
  fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
  fill(cache_traverse_.begin(), cache_traverse_.end(), -1);
  scene_wc_ = Eigen::Matrix4d::Zero();
  history_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  ROS_INFO("[SonarOccMap] Reset all buffers.");
}

Eigen::Vector3i SonarOccMap::getBufferSize() {
  return grid_size_;
}

void SonarOccMap::getOccupancy(std::vector<double> &occupancy_buffer) {
  occupancy_buffer = occupancy_buffer_;
}

inline bool SonarOccMap::isInLocalMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInLocalMap(idx);
}

inline bool SonarOccMap::isInLocalMap(const Eigen::Vector3i &id)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min_, min_id);
  posToIndex(local_range_max_, max_id);
  min_id(0) = max(0, min_id(0));
  min_id(1) = max(0, min_id(1));
  min_id(2) = max(0, min_id(2));
  max_id(0) = min(grid_size_[0], max_id(0));
  max_id(1) = min(grid_size_[1], max_id(1));
  max_id(2) = min(grid_size_[2], max_id(2));
  return (((id[0] - min_id[0]) | (max_id[0] - id[0]) | (id[1] - min_id[1]) | (max_id[1] - id[1]) | (id[2] - min_id[2]) | (max_id[2] - id[2])) >= 0);
};

bool SonarOccMap::isInMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInMap(idx);
}

bool SonarOccMap::isInMap(const Eigen::Vector3i &id)
{
  return ((id[0] | (grid_size_[0] - 1 - id[0]) | id[1] | (grid_size_[1] - 1 - id[1]) | id[2]| (grid_size_[2] - 1 - id[2])) >= 0);
};

void SonarOccMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void SonarOccMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  pos = origin_;
  for (int i = 0; i < 3; ++i)
    pos(i) += (id(i) + 0.5) * resolution_;
}

void SonarOccMap::setOccupancy(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  // cout << "id: " << id.transpose() << ", idx: " <<id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2) << ", is in map? " << isInMap(id) << endl;
  if (!isInMap(id))
    return;

  occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2)] = clamp_max_log_;
}

int SonarOccMap::getVoxelState(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return -1;
  if (!isInLocalMap(id))
    return 0;
  
  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2)] > min_occupancy_log_ ? 1 : 0;
}

int SonarOccMap::getVoxelState(const Eigen::Vector3i &id)
{
  if (!isInMap(id))
    return -1;
  if (!isInLocalMap(id))
    return 0;

  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2)] > min_occupancy_log_ ? 1 : 0;
}

void SonarOccMap::globalOccVisCallback(const ros::TimerEvent& e)
{
  //for vis
  history_view_cloud_ptr_->points.clear();
  for (int x = 0; x < grid_size_[0]; ++x)
    for (int y = 0; y < grid_size_[1]; ++y)
      for (int z = 0; z < grid_size_[2]; ++z)
      {
        //cout << "p(): " << occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] << endl;
        if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] > min_occupancy_log_)
        {
          Eigen::Vector3i idx(x,y,z);
          Eigen::Vector3d pos;
          indexToPos(idx, pos);
          pcl::PointXYZ pc(pos[0], pos[1], pos[2]);
          history_view_cloud_ptr_->points.push_back(pc);
        }
      }
  history_view_cloud_ptr_->width = history_view_cloud_ptr_->points.size();
  history_view_cloud_ptr_->height = 1;
  history_view_cloud_ptr_->is_dense = true;
  history_view_cloud_ptr_->header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*history_view_cloud_ptr_, cloud_msg);
  hist_view_cloud_pub_.publish(cloud_msg);
}

void SonarOccMap::sonarOdomCallback(const sensor_msgs::RangeConstPtr& sonar_msg, 
                              const nav_msgs::OdometryConstPtr& odom, 
                              const Eigen::Matrix4d& T_ic)
{
  /* ---------- get pose ---------- */
  // w, x, y, z -> q0, q1, q2, q3
  Eigen::Matrix3d R_wi = Eigen::Quaterniond(odom->pose.pose.orientation.w, 
                                            odom->pose.pose.orientation.x, 
                                            odom->pose.pose.orientation.y, 
                                            odom->pose.pose.orientation.z).toRotationMatrix();
  Eigen::Matrix4d T_wi;
  T_wi.setZero();
  T_wi(0, 3) = odom->pose.pose.position.x;
  T_wi(1, 3) = odom->pose.pose.position.y;
  T_wi(2, 3) = odom->pose.pose.position.z;
  T_wi(3, 3) = 1.0;
  T_wi.block<3,3>(0,0) = R_wi;
  Eigen::Matrix4d T_wc = T_wi * T_ic;
  Eigen::Vector3d t_wc = T_wc.block<3,1>(0,3);
  local_range_min_ = t_wc - sensor_range_;
	local_range_max_ = t_wc + sensor_range_;

  /* ---------- turn sonar message into pointcloud ---------- */
  proj_points_cnt_ = 0;
  sonarToPointCloud(sonar_msg->range, sonar_msg->field_of_view, T_wc, sonar_msg->header.stamp);
  raycastProcess(t_wc);

  local_map_valid_ = true;
  latest_odom_time_ = odom->header.stamp;
  curr_posi_[0] = odom->pose.pose.position.x;
  curr_posi_[1] = odom->pose.pose.position.y;
  curr_posi_[2] = odom->pose.pose.position.z;
  curr_twist_[0] = odom->twist.twist.linear.x;
  curr_twist_[1] = odom->twist.twist.linear.y;
  curr_twist_[2] = odom->twist.twist.linear.z;
  curr_q_.w() = odom->pose.pose.orientation.w;
  curr_q_.x() = odom->pose.pose.orientation.x;
  curr_q_.y() = odom->pose.pose.orientation.y;
  curr_q_.z() = odom->pose.pose.orientation.z;
  have_odom_ = true;
}

void SonarOccMap::sonarToPointCloud(const float range, const float field_of_view, const Eigen::Matrix4d& T_wc, ros::Time r_s)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointXYZRGB point; //colored point clouds also have RGB values

  // refer to https://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf
  float polar_rad = field_of_view / 2;
  float patch_area = 4 * M_PI / num_points_;
  int num_polar_partition = ceil(M_PI / sqrt(patch_area));
  float d_polar = M_PI / num_polar_partition;
  float d_azimuth = patch_area / d_polar;

  float theta, phi;
  int num_azimuth_partition;
  for (int i = 0; i < num_polar_partition; i++) {
    theta = M_PI * (i + 0.5) / num_polar_partition;
    if (theta > polar_rad) break;  // if larger than sonar field of fiew

    num_azimuth_partition = ceil(2 * M_PI * sin(theta) / d_azimuth);
    for (int j = 0; j < num_azimuth_partition; j++) {
      phi = 2 * M_PI * j / num_azimuth_partition;

      // create point using polar and azimuth angle
      Eigen::Vector3d proj_pt_NED, proj_pt_sonar;
      proj_pt_sonar(0) = range * sin(theta) * cos(phi);
      proj_pt_sonar(1) = range * sin(theta) * sin(phi);
      proj_pt_sonar(2) = range * cos(theta);
      proj_pt_NED = T_wc.block<3,3>(0,0) * proj_pt_sonar + T_wc.block<3,1>(0,3);
      proj_points_[proj_points_cnt_++] = proj_pt_NED;

      if (show_sonar_depth_)
      {
        point.x = proj_pt_NED[0];
        point.y = proj_pt_NED[1];
        point.z = proj_pt_NED[2];
        point.r = 255;
        point.g = 0;
        point.b = 0;
        cloud.points.push_back(point);
      }
    }
  }

  cloud.width = (int)cloud.points.size();
  cloud.height = 1;    //height=1 implies this is not an "ordered" point cloud
  // Convert the cloud to ROS message
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header.stamp = r_s;
  output.header.frame_id = "world";
  origin_pcl_pub_.publish(output);

  // // refer to http://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/
  // // for Golden Spiral algo
}

void SonarOccMap::raycastProcess(const Eigen::Vector3d& t_wc)
{
  if (proj_points_cnt_ == 0)
    return;

  // raycast_num_ = (raycast_num_ + 1) % 100000;
  raycast_num_ += 1;

  ROS_INFO_STREAM("proj_points_ size: " << proj_points_cnt_);

  /* ---------- iterate projected points ---------- */
  int set_cache_idx;
  for (int i = 0; i < proj_points_cnt_; ++i)
  {
    /* ---------- occupancy of ray end ---------- */
    Eigen::Vector3d pt_w = proj_points_[i];
    double length = (pt_w - t_wc).norm();
// 		ROS_INFO_STREAM("len: " << length);
    if (length < min_ray_length_)
      continue;
    else if (length > max_ray_length_)
    {
      // ROS_WARN("Cache Occupancy 0");
      pt_w = (pt_w - t_wc) / length * max_ray_length_ + t_wc;
      set_cache_idx = setCacheOccupancy(pt_w, 0);
    }
    else
      set_cache_idx = setCacheOccupancy(pt_w, 1);

    /* ---------- raycast will ignore close end ray ---------- */
    if (set_cache_idx != INVALID_IDX)
    {
      if (cache_rayend_[set_cache_idx] == raycast_num_)
      {
        continue;
      }
      else
        cache_rayend_[set_cache_idx] = raycast_num_;
    }

    //ray casting backwards from point in world frame to camera pos, 
    //the backwards way skips the overlap grids in each ray end by recording cache_traverse_.
    RayCaster raycaster;
    bool need_ray = raycaster.setInput(pt_w / resolution_, t_wc / resolution_); //(ray start, ray end)
    if (!need_ray)
      continue;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt;
    if (!raycaster.step(ray_pt)) // skip the ray start point since it's the projected point.
      continue;
    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
      set_cache_idx = setCacheOccupancy(tmp, 0);
      if (set_cache_idx != INVALID_IDX)
      {
        //skip overlap grids in each ray
        //NOTE: what if three occupied detection are on the same line?
        if (cache_traverse_[set_cache_idx] == raycast_num_)
          break;
        else
          cache_traverse_[set_cache_idx] = raycast_num_;
      }
    }
  }

  /* ---------- update occupancy in batch ---------- */
  while (!cache_voxel_.empty())
  {
    Eigen::Vector3i idx = cache_voxel_.front();
    int idx_ctns = idx(0) * grid_size_y_multiply_z_ + idx(1) * grid_size_(2) + idx(2);
    cache_voxel_.pop();

    double log_odds_update =
        cache_hit_[idx_ctns] >= cache_all_[idx_ctns] - cache_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;
    cache_hit_[idx_ctns] = cache_all_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_buffer_[idx_ctns] >= clamp_max_log_) continue;
    else if (log_odds_update <= 0 && occupancy_buffer_[idx_ctns] <= clamp_min_log_ - 1e-3) {
      occupancy_buffer_[idx_ctns] = clamp_min_log_;
      continue;
    }

    occupancy_buffer_[idx_ctns] =
        std::min(std::max(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_min_log_), clamp_max_log_);
  }
}

int SonarOccMap::setCacheOccupancy(const Eigen::Vector3d &pos, int occ)
{
  if (occ != 1 && occ != 0)
  {
    return INVALID_IDX;
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);

  if (!isInMap(id))
  {
    ROS_WARN_THROTTLE(1, "Point not inside map");
    return INVALID_IDX;
  }

  int idx_ctns = id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2);

  cache_all_[idx_ctns] += 1;

  if (cache_all_[idx_ctns] == 1)
  {
    cache_voxel_.push(id);
  }

  if (occ == 1)
    cache_hit_[idx_ctns] += 1;

  return idx_ctns;
}

double SonarOccMap::getMapDefault() {
  return clamp_min_log_ - unknown_flag_;
}

void SonarOccMap::init(const ros::NodeHandle& nh)
{
  node_ = nh;
  /* ---------- param ---------- */
  node_.param("sonar_occ_map/origin_x", origin_(0), -20.0);
  node_.param("sonar_occ_map/origin_y", origin_(1), -20.0);
  node_.param("sonar_occ_map/origin_z", origin_(2), 0.0);
  node_.param("sonar_occ_map/map_size_x", map_size_(0), 40.0);
  node_.param("sonar_occ_map/map_size_y", map_size_(1), 40.0);
  node_.param("sonar_occ_map/map_size_z", map_size_(2), 5.0);
  node_.param("sonar_occ_map/local_radius_x", sensor_range_(0), -1.0);
  node_.param("sonar_occ_map/local_radius_y", sensor_range_(1), -1.0);
  node_.param("sonar_occ_map/local_radius_z", sensor_range_(2), -1.0);
  node_.param("sonar_occ_map/resolution", resolution_, 0.2);

  node_.param("sonar_occ_map/num_points", num_points_, 5000);
  node_.param("sonar_occ_map/show_sonar_depth", show_sonar_depth_, true);
  
  node_.param("sonar_occ_map/min_ray_length", min_ray_length_, -0.1);
  node_.param("sonar_occ_map/max_ray_length", max_ray_length_, -0.1);

  node_.param("sonar_occ_map/p_hit", p_hit_, 0.70);
  node_.param("sonar_occ_map/p_miss", p_miss_, 0.35);
  node_.param("sonar_occ_map/p_min", p_min_, 0.12);
  node_.param("sonar_occ_map/p_max", p_max_, 0.97);
  node_.param("sonar_occ_map/p_occ", p_occ_, 0.80);

  prob_hit_log_ = logit(p_hit_);
  prob_miss_log_ = logit(p_miss_);
  clamp_min_log_ = logit(p_min_);
  clamp_max_log_ = logit(p_max_);
  min_occupancy_log_ = logit(p_occ_);

  cout << "map size: " << map_size_.transpose() << endl;
  cout << "resolution: " << resolution_ << endl;

  cout << "hit: " << prob_hit_log_ << endl;
  cout << "miss: " << prob_miss_log_ << endl;
  cout << "min: " << clamp_min_log_ << endl;
  cout << "max: " << clamp_max_log_ << endl;
  cout << "thresh: " << min_occupancy_log_ << endl;
	cout << "sensor_range: " << sensor_range_.transpose() << endl;

  /* ---------- setting ---------- */
  have_odom_ = false;
  global_map_valid_ = false;
  local_map_valid_ = false;
  has_global_cloud_ = false;
  has_first_depth_ = false;
  buffer_reset_ = true;
  
  // Init scene center
  scene_wc_ = Eigen::Matrix4d::Zero();

  resolution_inv_ = 1 / resolution_;
  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_);

  history_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Current T_ic0_ corresponds to sonar on dragon_ddk model
  T_ic0_ << 0.0,  0.0, 1.0, 0.05,
            -1.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.05,
            0.0,  0.0, 0.0, 1.0;
								
  cout << "origin_: " << origin_.transpose() << endl;
  min_range_ = origin_;
  max_range_ = origin_ + map_size_;
  cout << "min_range_: " << min_range_.transpose() << endl;
  cout << "max_range_: " << max_range_.transpose() << endl;

  //init proj_points_ buffer
  proj_points_.resize(num_points_ / 30);  // num_points_ = 300000 will results in 5728 actual points in pointcloud
								
  // initialize size of buffer
  grid_size_y_multiply_z_ = grid_size_(1) * grid_size_(2);
  int buffer_size = grid_size_(0) * grid_size_y_multiply_z_;
  cout << "buffer size: " << buffer_size << endl;
  occupancy_buffer_.resize(buffer_size);

  cache_all_.resize(buffer_size);
  cache_hit_.resize(buffer_size);

  cache_rayend_.resize(buffer_size);
  cache_traverse_.resize(buffer_size);
  raycast_num_ = 0;
  
  proj_points_cnt_ = 0;
  unknown_flag_ = 0.01;
  fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), clamp_min_log_ - unknown_flag_);

  fill(cache_all_.begin(), cache_all_.end(), 0);
  fill(cache_hit_.begin(), cache_hit_.end(), 0);

  fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
  fill(cache_traverse_.begin(), cache_traverse_.end(), -1);

  /* ---------- sub and pub ---------- */
  sonar_sub_.reset(new message_filters::Subscriber<sensor_msgs::Range>(node_, "/sonar_topic", 1, ros::TransportHints().tcpNoDelay()));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/odom_world", 1, ros::TransportHints().tcpNoDelay()));
  sync_sonar_odom_.reset(new message_filters::Synchronizer<SyncPolicySonarOdom>(SyncPolicySonarOdom(100), *sonar_sub_, *odom_sub_));
  sync_sonar_odom_->registerCallback(boost::bind(&SonarOccMap::sonarOdomCallback, this, _1, _2, T_ic0_));
  global_occ_vis_timer_ = node_.createTimer(ros::Duration(0.5), &SonarOccMap::globalOccVisCallback, this);

  hist_view_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sonar_occ_map/history_view_cloud", 1);
	origin_pcl_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sonar_occ_map/raw_pcl", 1);
}
