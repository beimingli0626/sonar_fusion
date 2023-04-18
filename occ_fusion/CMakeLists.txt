cmake_minimum_required(VERSION 3.10)
project(occ_fusion)

ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## equivalent to -L /usr/local/lib of gcc/g++
## Then libcnpy.so will be used as cnpy

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  cv_bridge
  occ_grid
  sonar_occ_grid
  occ_fusion_msgs
  message_generation
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  nav_msgs
  visualization_msgs
  geometry_msgs
  cv_bridge
  occ_grid
  sonar_occ_grid
  occ_fusion_msgs
  message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  # /usr/local/include
)
# cnpy.h is inside /usr/local/include

## Declare a cpp library
add_library(occ_fusion 
    src/occ_fusion.cpp 
)

## Declare a cpp executable
add_executable(occ_fusion_node
    src/occ_fusion_node.cpp
    src/occ_fusion.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(occ_fusion_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(occ_fusion_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(occ_fusion 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)