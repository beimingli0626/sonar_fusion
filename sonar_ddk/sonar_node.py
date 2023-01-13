#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Range
from serialAPI import SerialAPI

UART_J7 = 9
UART_J10 = 7
UART_J11 = 12
UART_J12 = 5

BAUD_RATE = 9600

def talker(serialAPI):
    pub = rospy.Publisher('sonar_topic', Range, queue_size=10)
    rospy.init_node('sonar_node', anonymous=True)
    rate = rospy.Rate(15)
    seq = 0
    while not rospy.is_shutdown():
        distances = serialAPI.read()
        if len(distances) == 0:
            continue    # we are reading faster than the update speed of Sonar, might get 0 data
        sonar_msg = toRange(distances, seq)
        rospy.loginfo(sonar_msg)
        pub.publish(sonar_msg)
        rate.sleep()


def toRange(distances: list, seq: int) -> Range:
    distance = distances[0] # assume we only get 1 datapoint per timestep
    seq += 1
    sonar_msg = Range()

    sonar_msg.header.seq = seq
    sonar_msg.header.stamp = rospy.Time.now()
    sonar_msg.header.frame_id = "sonar"

    sonar_msg.radiation_type = Range.ULTRASOUND
    sonar_msg.field_of_view = 0.4
    sonar_msg.min_range = 0.2
    sonar_msg.max_range = 7.5
    sonar_msg.range = float(distance) / 100
    return sonar_msg

  
if __name__ == '__main__':
    try:
        serialAPI = SerialAPI(UART_J10, BAUD_RATE)
        talker(serialAPI)
    except rospy.ROSInterruptException:
        pass