#ifndef _SONAR_OCC_MAP_H
#define _SONAR_OCC_MAP_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <tf2_ros/transform_listener.h>

#include <queue>
#include <cmath>

#define logit(x) (log((x) / (1 - (x))))
#define INVALID_IDX -1

using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::string;
using std::queue;
using std::shared_ptr;
using std::max;
using std::min;
using std::floor;
using std::ceil;
using std::isnan;
using std::isinf;
using std::cos;
using std::sin;
using std::sqrt;
using std::acos;
using std::pow;


class SonarOccMap
{
public:
  SonarOccMap() {}
  ~SonarOccMap() {}
  void init(const ros::NodeHandle& nh);

  bool odomValid() { return have_odom_; }
  bool mapValid() { return (global_map_valid_||local_map_valid_); }
  Eigen::Vector3d get_curr_posi() { return curr_posi_; }
	Eigen::Vector3d get_curr_twist() {return curr_twist_; }
  Eigen::Vector3d get_curr_acc() {return curr_acc_; }
  Eigen::Quaterniond get_curr_quaternion() {return curr_q_; }
  double getResolution() { return resolution_; }
  double getOccThreshold() {return min_occupancy_log_;}
  double getClampMinLog() {return clamp_min_log_;}
  Eigen::Vector3d getOrigin() { return origin_; }
  Eigen::Vector3d getMapSize() { return map_size_; }
  Eigen::Matrix4d getSceneTF() {return scene_wc_; }

  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
  void resetAllBuffer();
  Eigen::Vector3i getBufferSize();
  void getOccupancy(std::vector<double> &occupancy_buffer);
  void setOccupancy(const Eigen::Vector3d &pos);
  int getVoxelState(const Eigen::Vector3d &pos);
  int getVoxelState(const Eigen::Vector3i &id);
	ros::Time getLocalTime() { return latest_odom_time_; };

  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d& pos);

  bool isInMap(const Eigen::Vector3d &pos);
  bool isInMap(const Eigen::Vector3i &id);

  double getMapDefault();
  typedef shared_ptr<SonarOccMap> Ptr;
  
private:
  std::vector<double> occupancy_buffer_;  // 0 is free, 1 is occupied

  // map property
  Eigen::Vector3d min_range_, max_range_;  // map range in pos
  Eigen::Vector3i grid_size_;              // map size in index
  int grid_size_y_multiply_z_;
  Eigen::Vector3d local_range_min_, local_range_max_;

  bool isInLocalMap(const Eigen::Vector3d &pos);
  bool isInLocalMap(const Eigen::Vector3i &id);

  Eigen::Vector3d origin_, map_size_;
  double resolution_, resolution_inv_;
  int num_points_;
  Eigen::Matrix4d T_ic0_, T_ic1_, T_ic2_, T_ic3_;

  bool have_odom_;
	Eigen::Vector3d curr_posi_, curr_twist_, curr_acc_;
	Eigen::Quaterniond curr_q_;

  // ros
  ros::NodeHandle node_;
  ros::Timer global_occ_vis_timer_;
	
  // for vis
	ros::Time latest_odom_time_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr history_view_cloud_ptr_;
	ros::Publisher hist_view_cloud_pub_; 
	ros::Publisher pose_vis_pub_, twist_vis_pub_, acc_vis_pub_;

  // for sonar
  double epsilon_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, nav_msgs::Odometry> SyncPolicySonarOdom;
	typedef shared_ptr<message_filters::Synchronizer<SyncPolicySonarOdom>> SynchronizerSonarOdom;
  SynchronizerSonarOdom sync_sonar_odom_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Range>> sonar_sub_;
  void sonarOdomCallback(const sensor_msgs::RangeConstPtr& sonar_msg, 
                         const nav_msgs::OdometryConstPtr& odom, 
                         const Eigen::Matrix4d& T_ic);
  void sonarToPointCloud(const float range, const float field_of_view, const Eigen::Matrix4d& T_wc, ros::Time r_s);
  void raycastProcess(const float range, const float field_of_view, const Eigen::Matrix4d& T_wc);
  double getSonarLogOddsUpdate(const float range, const float field_of_view, const Eigen::Vector3i& center_idx, const Eigen::Vector3i& odom_idx, const Eigen::Vector3i& idx);
  int setCacheOccupancy(const Eigen::Vector3d &pos, int occ);

  void globalOccVisCallback(const ros::TimerEvent& e);

  bool has_global_cloud_, has_first_depth_;
	bool global_map_valid_, local_map_valid_;

  // map fusion 
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt_;
  vector<int> cache_hit_, cache_all_;
  vector<int> cache_traverse_, cache_rayend_;
  int raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;
  Eigen::Vector3d sensor_range_;
  Eigen::Matrix4d scene_wc_;
  bool show_sonar_depth_;
  bool fully_initialized_;
  bool buffer_reset_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_;
	double min_occupancy_log_;
  double min_ray_length_, max_ray_length_;
  double unknown_flag_;
  
	/* origin pcl show */
	ros::Publisher origin_pcl_pub_;
};


#endif
