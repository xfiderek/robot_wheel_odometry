#!/usr/bin/env python
import rospy 
from robot_wheel_odometry.msg import WheelShift, WheelShifts
import numpy as np

if __name__ == "__main__":
    rospy.init_node('drive_straight')
    pub = rospy.Publisher("/wheel_shifts", WheelShifts, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        shifts = WheelShifts()

        shift = WheelShift()
        shift.distance = 0.1 
        shift.wheel_name.data = "fr" 
        shifts.shifts.append(shift)

        shift = WheelShift()
        shift.distance = 0.1 
        shift.wheel_name.data = "fl" 
        shifts.shifts.append(shift)

        shift = WheelShift()
        shift.distance = 0.1 
        shift.wheel_name.data = "bl" 
        shifts.shifts.append(shift)

        shift = WheelShift()
        shift.distance = 0.1
        shift.wheel_name.data = "br" 
        shifts.shifts.append(shift)

        pub.publish(shifts)
        rate.sleep()
        