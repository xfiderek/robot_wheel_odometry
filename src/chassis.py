import numpy as np 
from utils import Pose2d, Point2d
import rospy 


class Chassis:
    def __init__(self):        
        wheels_cfg = rospy.get_param("~wheels/")
        wheels_names = wheels_cfg.keys()

        #dictionary that contains objects of Wheel class defined below created with data from wheels config file
        self.wheels = {name : Wheel(wheels_cfg[name]["x"], wheels_cfg[name]["y"]) \
                       for name in wheels_names
                      }
        self.last_state_stamp = rospy.Time()               
        self.last_wheels_involved = [] #names of wheels that were involved in last obtained message. defining this field makes it possible to calculate 
                                    #odometry even though some wheels defined in config had not provided data in this iteration

    def updateState(self, wheel_shifts_msg):
        shifts = wheel_shifts_msg.shifts
        self.last_wheels_involved = []
        self.last_state_stamp = wheel_shifts_msg.header.stamp 
        for shift in shifts:            
            try:
                wheel = self.wheels[shift.wheel_name.data]
                wheel.updateWheelState(shift.distance, shift.angle)
                self.last_wheels_involved.append(shift.wheel_name.data)
            except KeyError:
                raise KeyError("WHEEL OF NAME :{} IS NOT DEFINED IN CONFIG FILE".format(shift.wheel_name.data))
    
    def getWheels(self):
        return self.wheels
    

class Wheel:
    def __init__(self, initial_x, initial_y):
        self.angle = 0.
        self.last_traveled_distance = 0.
        self.initial_pose = Point2d(initial_x, initial_y)

    #state with respect to previous iteration
    def updateWheelState(self, distance, angle):
        self.last_traveled_distance = distance
        self.angle = angle 

    def getLastShift(self):
        shift_x = self.last_traveled_distance * np.cos(self.angle)
        shift_y = self.last_traveled_distance * np.sin(self.angle)
        shift = Point2d(shift_x, shift_y)
        
        return shift



