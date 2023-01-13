# sonar_vis
This repository is to visualize and postprocess bags recorded on the ddk drone.

### Usage
- `roslaunch sonar_vis ddk_rviz.launch` for starting rviz
- `roslaunch sonar_vis sonar_tf.launch` for publishing static transform from sonar frame to drone baselink
- `roslaunch sonar_vis octomap_mapping.launch` for starting octomap server