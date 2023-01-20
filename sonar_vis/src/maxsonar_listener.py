#! /usr/bin/env python
#from __future__ import print_function

import rospy
import numpy as np
import random
import math
import sys

import tf
import sensor_msgs
import std_msgs

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_msgs.msg import Header
#from std_msgs.msg import Image
import locale
from locale import atof

import serial
#from serial import *
from std_msgs.msg import String

pc2_list = []

def publishPC2(srange):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "sonar"
    header.stamp = rospy.Time.now()

    dim = 1.0
    u, v = np.meshgrid(np.linspace(-dim/50,dim/50,200), np.linspace(-dim/50,dim/50,200))
    x = u * srange * 8
    y = v * srange * 8
    z = u + srange
    points = np.array([z,y,x,u]).reshape(4,-1).T

    pc2 = point_cloud2.create_cloud(header, fields, points)
    return pc2


def callback(data):
    pc2_list.append(data.range)


def listener():
    rospy.init_node('pc2_publisher')
    pub = rospy.Publisher('dragonfly26/tof/voxl_point_cloud', PointCloud2, queue_size=100)
    rate = rospy.Rate(10)
    rospy.Subscriber("sonar_topic", Range, callback)
    while not rospy.is_shutdown():
        if len(pc2_list) > 0:
            cloud_out = publishPC2(pc2_list[0]) 
            pc2_list.pop(0)
            pub.publish(cloud_out)
        #rate.sleep()


if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
