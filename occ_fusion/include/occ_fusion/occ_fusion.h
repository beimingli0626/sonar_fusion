#ifndef DEPTH_OCC_H
#define DEPTH_OCC_H

#include <ros/ros.h>
#include "occ_grid/occ_map.h"
#include "sonar_occ_grid/sonar_occ_map.h"


class OccFusion {
  public:
    OccFusion(ros::NodeHandle &nh);
    ~OccFusion();
    
  private:
    OccMap::Ptr occ_map_;
    SonarOccMap::Ptr sonar_occ_map_;
};


#endif
