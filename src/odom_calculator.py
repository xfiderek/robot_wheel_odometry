from utils import Point2d, Pose2d
import rospy 
import numpy as np 
from nav_msgs.msg import Odometry 

from displacement_calculator import DisplacementCalculator
from utils import InsufficientDataException

class OdometryCalculator:
    def __init__(self, chassis):
        self.chassis = chassis
        self.last_pose = Pose2d()
        self.odom_frame = rospy.get_param("~odom_frame")
        self.displacement_calc = DisplacementCalculator(chassis, 
                                            rospy.get_param("~filter_noisy_wheels"), 
                                            rospy.get_param("~noisy_wheels_filter_threshold", default=np.inf),
                                            rospy.get_param("~min_number_of_reliable_wheels", default=2)
                                            )

    def calcOdom(self):
        odom = Odometry()
        odom.header.frame_id = self.odom_frame
        odom.header.stamp = self.chassis.last_state_stamp
        odom.child_frame_id = 'wheel_odom'
    
        try:        
            pose_displacement_2d = self.estimateChangeOfPose()
            self.updateLastPose(pose_displacement_2d)
            odom.pose.pose = self.last_pose.toPoseStamped().pose
        
        except InsufficientDataException as ex:
            rospy.logwarn(ex.message)
            odom.pose.covariance = self.infiniteCovariance()
            odom.twist.covariance = self.infiniteCovariance()

        return odom 

    def estimateChangeOfPose(self):
        pose_displacement_2d = Pose2d()
        dx, dy, dtheta = self.displacement_calc.calculateDisplacement()
        pose_displacement_2d.position = Point2d(dx, dy)
        pose_displacement_2d.angle = dtheta
        
        return pose_displacement_2d 

    def updateLastPose(self, pose_displacement_2d):
        self.last_pose.position += pose_displacement_2d.position.rotate(self.last_pose.angle)
        self.last_pose.angle += pose_displacement_2d.angle 

    def resetPose(self):
        self.last_pose = Pose2d()

    def infiniteCovariance(self):
        cov_matrix = np.zeros(shape=(6, 6))
        for i in range(6):
            cov_matrix[i, i] = np.inf
        return cov_matrix.reshape(1,36).tolist()[0]