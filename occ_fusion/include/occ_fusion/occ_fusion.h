#ifndef DEPTH_OCC_H
#define DEPTH_OCC_H

#include <ros/ros.h>
#include "occ_grid/occ_map.h"
#include "sonar_occ_grid/sonar_occ_map.h"
#include <occ_fusion_msgs/DetectWindow.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>

class OccFusion {
  public:
    OccFusion(ros::NodeHandle &nh);
    ~OccFusion();
    
  private:
    std::vector<int8_t> cam_occupancy_buffer_;
    OccMap::Ptr occ_map_;
    SonarOccMap::Ptr sonar_occ_map_;

    ros::Timer cam_occ_update_timer_;
    ros::Timer detect_window_timer_;
    ros::NodeHandle node_;

    void camOccMapUpdateCallback(const ros::TimerEvent& e);

    // window detection
    Eigen::Vector3d origin_, map_size_;
    Eigen::Vector3i grid_size_;              // map size in index
    double resolution_, resolution_inv_;
    double x_search_range_, y_search_range_;
    void detectWindowCallback(const ros::TimerEvent& e);
    void detectOpen(float coeff_x, float coeff_y, float coeff_const, Eigen::Vector3i &sonar_center_idx);
    void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);

    /* plane segmentation pcl show */
    bool vis_plane_pcl_;
	  ros::Publisher plane_pcl_pub_, window_marker_pub_;
    int window_marker_id_;

    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;

    // ros::ServiceServer detect_window_server_;
    // bool detectWindow(occ_fusion_msgs::DetectWindow::Request &req, 
    //                   occ_fusion_msgs::DetectWindow::Response &res);
};


#endif
