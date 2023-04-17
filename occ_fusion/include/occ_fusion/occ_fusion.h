#ifndef DEPTH_OCC_H
#define DEPTH_OCC_H

#include <ros/ros.h>
#include "occ_grid/occ_map.h"
#include "sonar_occ_grid/sonar_occ_map.h"
#include <occ_fusion_msgs/DetectWindow.h>


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
    void detectWindowCallback(const ros::TimerEvent& e);

    // ros::ServiceServer detect_window_server_;
    // bool detectWindow(occ_fusion_msgs::DetectWindow::Request &req, 
    //                   occ_fusion_msgs::DetectWindow::Response &res);
};


#endif
