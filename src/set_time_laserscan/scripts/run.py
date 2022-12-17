#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class Modifier:
    def __init__(self):
        self.time_sec_scan = None
        self.time_nsec_scan = None

        self.prev_time = None
        self.cur_time = None

        self.new_laserscan = LaserScan()
        
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

        rospy.init_node('set_scan_time', anonymous=True)
        self.sub1 = rospy.Subscriber('/merged_scan', LaserScan, self.scan_merged_callback)

        self.prev_time = rospy.Time.now()

        rospy.spin()

    def scan_merged_callback(self, msg):
        self.new_laserscan.header.seq = msg.header.seq
        self.new_laserscan.header.stamp = rospy.Time.now()
        self.new_laserscan.header.frame_id = "base_scan"
        self.new_laserscan.angle_min = msg.angle_min
        self.new_laserscan.angle_max = msg.angle_max
        self.new_laserscan.angle_increment = msg.angle_increment
        self.new_laserscan.time_increment = msg.scan_time / ((msg.angle_max - msg.angle_min) / msg.angle_increment)
        self.new_laserscan.scan_time = msg.scan_time
        self.new_laserscan.range_min = msg.range_min
        self.new_laserscan.range_max = msg.range_max
        self.new_laserscan.ranges = msg.ranges
        self.new_laserscan.intensities = msg.intensities

        self.pub.publish(self.new_laserscan)
       

if __name__ == '__main__':
    modifier = Modifier()
