#!/usr/bin/env python

import rospy
import sensor_msgs.msg

count = 0
def callback(data):
    global count
    count = count + 1
    print("get_samples: ", count, len(data.ranges))
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/scan_1", sensor_msgs.msg.LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()