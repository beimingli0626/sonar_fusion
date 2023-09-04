# sonar_fusion

This repo contains code for sensor fusion of time-of-flight camera and ultrasonic sensor.

## submodules
- sonar_ddk: to be cloned onto the ddk drone for reading sonar messages and publish as rostopic for later use
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
