# Sonar Fusion

We investigate the challenge posed by glass windows, which are common components in indoor environments, as they are not detectable by depth cameras. In contrast, ultrasonic sensors emit high-pitched sound pulses that are reflected regardless of lighting conditions or object's optical properties. Therefore, we investigate the feasibility of integrating ultrasonic sensors as a secondary sensing modality to our current mapping stack. The outcomes of this independent study include the development of simulation environments in Gazebo to replicate indoor building with windows, the implementation of a sensor fusion mechanism for combining depth image and sonar data, and the design of a RANSAC-based approach for window segmentation in indoor exploration scenarios. These outcomes enable the robot to recognize glass as obstacles and identify potential window regions based on geometric characteristics. This study provides insights into potential benefits and limitations of sensor fusion of depth camera and ultrasonic sensor.

## Submodules
- sonar_ddk: to be cloned onto the drone for reading sonar messages through UART and publish as rostopic for later use
- occ_grid: a occupancy grid library with occupancy updating based on raw depth image and raytracing algorithm
- sonar_occ_grid: a occupancy grid library with occupancy updating based on sonar range message and raytracing algorithm
- occ_fusion: fusing the sonar-based occupancy grid and depth image based occupancy grid
- sonar_vis: to visualize and postprocess bags recorded on the ddk drone

## Instructions for running

This repository is supposed to be run with the Kumar Robotics Exploration stack. For details on the installation of the Exploration stack, refer to
```Exploration_stack_installation.txt``` included in this repository.

- Navigate to "scripts" folder in ```ddk_sim_launch```
```
roscd ddk_sim_launch/scripts
```

- run the ```demo_ddk.sh``` file
```
./demo_ddk.sh
```

- After Gazeb and rviz are loaded, On another terminal, navigate to "launch" folder in ```occ_predictor_2d``` and run ``roslaunch occ_to_img.launch``
```
roscd occ_predictor_2d/launch
roslaunch occ_to_img.launch
```
