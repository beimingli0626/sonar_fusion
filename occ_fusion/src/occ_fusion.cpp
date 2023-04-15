#include <occ_fusion/occ_fusion.h>


OccFusion::OccFusion(ros::NodeHandle &nh) {
  occ_map_.reset(new OccMap);
  occ_map_->init(nh);
  sonar_occ_map_.reset(new SonarOccMap);
  sonar_occ_map_->init(nh);
}

OccFusion::~OccFusion() {}
