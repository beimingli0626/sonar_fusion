#include <ros/ros.h>
#include "occ_fusion/occ_fusion.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "occ_fusion_node");
  ros::NodeHandle nh("~");

  OccFusion occ_fusion(nh);

  ros::spin();
  return 0;
}