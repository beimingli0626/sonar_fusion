#include <occ_fusion/occ_fusion.h>


OccFusion::OccFusion(ros::NodeHandle &nh) {
  node_ = nh;
  occ_map_.reset(new OccMap);
  occ_map_->init(nh);
  sonar_occ_map_.reset(new SonarOccMap);
  sonar_occ_map_->init(nh);

  cam_occ_update_timer_ = node_.createTimer(ros::Duration(0.2), &OccFusion::camOccMapUpdateCallback, this);
}

OccFusion::~OccFusion() {}

void OccFusion::camOccMapUpdateCallback(const ros::TimerEvent& e){
  std::vector<std::int8_t> occupancy;
  occ_map_->getOccupancyBinary(occupancy);
  sonar_occ_map_->updateCamOccupancy(occupancy);
}
