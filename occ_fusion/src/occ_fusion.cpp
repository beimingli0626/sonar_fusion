#include <occ_fusion/occ_fusion.h>


OccFusion::OccFusion(ros::NodeHandle &nh) 
{
  node_ = nh;
  occ_map_.reset(new OccMap);
  occ_map_->init(nh);
  sonar_occ_map_.reset(new SonarOccMap);
  sonar_occ_map_->init(nh);

  cam_occ_update_timer_ = node_.createTimer(ros::Duration(0.2), &OccFusion::camOccMapUpdateCallback, this);
  detect_window_timer_ = node_.createTimer(ros::Duration(0.2), &OccFusion::detectWindowCallback, this);
  
  // detect_window_server_ = node_.advertiseService("/occ_fusion/detect_window", &OccFusion::detectWindow, this);
}

OccFusion::~OccFusion() {}

void OccFusion::camOccMapUpdateCallback(const ros::TimerEvent& e)
{
  occ_map_->getOccupancyBinary(cam_occupancy_buffer_);
  sonar_occ_map_->updateCamOccupancy(cam_occupancy_buffer_);
}

void OccFusion::detectWindowCallback(const ros::TimerEvent& e)
{
  Eigen::Vector3i sonar_center_idx = sonar_occ_map_->getSonarCenterIndex();
}

// bool OccFusion::detectWindow(occ_fusion_msgs::DetectWindow::Request &req, 
//                             occ_fusion_msgs::DetectWindow::Response &res)
// {
//   ROS_INFO("[Occ Fusion] Receive Detect Window service call");
//   res.res_str = "Detect window success";
//   return true;
// }