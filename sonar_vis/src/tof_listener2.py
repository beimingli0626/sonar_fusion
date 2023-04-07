#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

def callback(data):
    # Convert the point cloud message to a list of points
    point_list = list(pc2.read_points(data))

    widi = data.height
    hidi = data.width
    resh = np.zeros((widi, hidi))
    mm = widi // 2
    yy = hidi // 2
    lf = 1.5
    rf = 2.3
    for i in range(resh.shape[0]):
        for j in range(resh.shape[1]):
            #print(points[i])
            if i > mm - (mm // lf) and i < mm + (mm // lf) and j > yy - (yy // rf) and j < yy + (yy // rf):
                resh[i][j] = 1

    resh = resh.flatten()

    for ii in reversed(range(resh.shape[0])):
        if (resh[ii] == 1):
            del point_list[ii]
            #points_list = points_list[:ii]

    noise = 0.042
    for i in reversed(range(len(point_list))):
        if np.float(point_list[i][3]) > noise:
            del point_list[i]

    # Remove 10 points from the list
    #point_list = point_list[:-10]

    # Create a new point cloud message with the remaining points
    new_cloud = pc2.create_cloud(data.header, data.fields, point_list)

    # Publish the new point cloud message
    pub.publish(new_cloud)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('point_cloud_filter')

    # Create a subscriber for the point cloud topic
    sub = rospy.Subscriber('dragonfly26/tof/voxl_point_cloud', PointCloud2, callback)

    # Create a publisher for the new point cloud topic
    pub = rospy.Publisher('dragonfly26/fusion', PointCloud2, queue_size=10)

    # Spin the node
    rospy.spin()