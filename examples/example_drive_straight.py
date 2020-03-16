#!/usr/bin/env python
import rospy 
from robot_wheel_odometry.msg import WheelShift, WheelShifts

shifts = WheelShifts()

shift_fr = WheelShift()
shift_fr.distance = 0.1 
shift_fr.wheel_name.data = "fr" 

shift_fl = WheelShift()
shift_fl.distance = 0.1 
shift_fl.wheel_name.data = "fl" 

shift_bl = WheelShift()
shift_bl.distance = 0.1 
shift_bl.wheel_name.data = "bl" 

shift_br = WheelShift()
shift_br.distance = 0.1
shift_br.wheel_name.data = "br" 
    
shifts.shifts.append(shift_fr)
shifts.shifts.append(shift_fl)
shifts.shifts.append(shift_bl)
shifts.shifts.append(shift_br)

if __name__ == "__main__":
    rospy.init_node('drive_straight')
    pub = rospy.Publisher("/wheel_shifts", WheelShifts, queue_size=1)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        pub.publish(shifts)
        rate.sleep()
        
