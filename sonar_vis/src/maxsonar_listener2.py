#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np

def callback(data):

    srange = data.range

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "sonar"
    header.stamp = rospy.Time.now()

    dim = 1.0/50 #size of sonar beam
    xdim = np.linspace(-dim,dim,srange * 8)
    ydim = np.linspace(-dim,dim,srange * 8)
    #xdim = np.linspace(0,0.5,90)*pi/180
    #ydim = xdim-min(xdim)
    u, v = np.meshgrid(xdim,ydim)
    x = u * srange * 8
    y = v * srange * 8
    z = u + srange

    points = np.array([z,y,x,-u]).reshape(4,-1).T
    #print("LEN: ", points.shape)
    target_area = np.sqrt(points.shape[0])
    #print("PTSS: ", points.shape[0])
    #print(target_area)

    r = int((target_area * 0.5))
    #print("rad: ", r)
    m = np.zeros((2*r,2*r))
    #print("ms: ", m.shape)

    a, b = r, r

    for row in range(0, m.shape[0]):
        for col in range(0, m.shape[1]):
            if (col-a)**2 + (row-b)**2 <= (r-1)**2:
                m[row,col] = 1

    np.sum(m)
    #print(m)
    m = m.T
    #print("T", m)
    m = m.flatten()
    #print("F;atten: ", m.shape)
    for i in reversed(range(0,m.shape[0])):
        if (m[i] == 0):
            points = np.delete(points, i, 0)

    # Create a new point cloud message with the remaining points
    new_cloud = pc2.create_cloud(header, fields, points)

    # Publish the new point cloud message
    pub.publish(new_cloud)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('sonar_cloud_builder')

    # Create a subscriber for the point cloud topic
    sub = rospy.Subscriber('sonar_topic', Range, callback)

    # Create a publisher for the new point cloud topic
    pub = rospy.Publisher('dragonfly26/fusion', PointCloud2, queue_size=10)

    # Spin the node
    rospy.spin()