
import numpy as np 
import rospy 
from utils import InsufficientDataException

class DisplacementCalculator:
    def __init__(self, chassis, filter_noisy_wheels, noisy_wheels_filter_threshold, min_number_of_reliable_wheels):
        self.filter_noisy_wheels = filter_noisy_wheels
        self.error_threshold = noisy_wheels_filter_threshold
        self.reliable_wheels_threshold = min_number_of_reliable_wheels
        self.chassis = chassis 
        self.wheels = chassis.getWheels()        
    
    def calculateDisplacement(self):
        pose_estimation_vector = []
        pose_estimation_matrix = []
        
        last_wheels_involved = self.chassis.last_wheels_involved 
        if len(last_wheels_involved) < self.reliable_wheels_threshold:
            raise InsufficientDataException("too many wheels have not provided data in this iteration")
        
        if self.filter_noisy_wheels:
            removed_wheel = self.removeNoisiestWheel()
            while removed_wheel and len(last_wheels_involved) > 0:
                removed_wheel = self.removeNoisiestWheel()

            if len(last_wheels_involved) < self.reliable_wheels_threshold:    
                raise InsufficientDataException("data from wheels was too noisy to calculate reliable odometry")
        
        for wh_name in last_wheels_involved:
            wheel = self.wheels[wh_name]
            wheel_shift = wheel.getLastShift()
            pose_estimation_vector += [wheel_shift.x, wheel_shift.y]                
            pose_estimation_matrix.append([1, 0, -wheel.initial_pose.y])
            pose_estimation_matrix.append([0, 1,  wheel.initial_pose.x])
                
        dx, dy, dtheta = np.linalg.lstsq(np.array(pose_estimation_matrix), np.array(pose_estimation_vector).T, rcond=None)[0]
        return dx, dy, dtheta

    def removeNoisiestWheel(self):
        errors = {}
        removed_wheel = False 
        
        for wh_name in self.chassis.last_wheels_involved:
            error = 0.
            wheel = self.wheels[wh_name]
            wheel_shift = wheel.getLastShift()
        
            for other_wh_name in self.chassis.last_wheels_involved:
                if wh_name == other_wh_name: 
                    continue 

                second_wheel = self.wheels[other_wh_name] 
                second_wheel_shift = second_wheel.getLastShift()
                link_between_wheels = wheel.initial_pose - second_wheel.initial_pose
                wheels_relative_displacement = wheel_shift - second_wheel_shift
                rel_displcmnt_in_link_direction = wheels_relative_displacement.dot(link_between_wheels) / link_between_wheels.norm()
                
                error += np.abs(rel_displcmnt_in_link_direction / wheel.last_traveled_distance)

            if error > self.error_threshold:
                errors[wh_name] = error 
            print wh_name, error
        if errors:
            noisiest_wheel = max(errors, key=errors.get) #it returns key of most noisy wheel
            self.chassis.last_wheels_involved.remove(noisiest_wheel)
            removed_wheel = True 

        return removed_wheel

