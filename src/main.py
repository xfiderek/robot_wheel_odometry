#!/usr/bin/env python
import rospy 
from odom_publisher import OdometryPublisher

if __name__ == "__main__":
    OdometryPublisher()
    rospy.spin()