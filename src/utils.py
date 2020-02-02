import numpy as np 
from geometry_msgs.msg import PoseStamped
import tf 

class InsufficientDataException(Exception):
    pass 

class Point2d(object):
    def __init__(self, x=0., y=0.):
        self.x = x
        self.y = y

    def norm(self):
        return np.sqrt(self.x**2+self.y**2)

    def dot(self, point):
        return (self.x * point.x + self.y * point.y)

    def rotate(self, angle):
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], 
                               [np.sin(angle), np.cos(angle)]])
   
        vec_rotated = np.dot(rotation_matrix, self.toVector())
        point_rotated = self.fromVector(vec_rotated)

        return point_rotated
    
    def toVector(self):
        return np.array([self.x, self.y])
    
    def fromVector(self, vec):
        point = Point2d(vec[0], vec[1])
        return point 

    def __add__(self, point):
        new_point = Point2d(self.x + point.x,self.y + point.y)
        return new_point 

    def __sub__(self, point):
        new_point = Point2d(self.x - point.x, self.y - point.y)
        return new_point 
    
    def __rmul__(self, number):
        point = Point2d(self.x*number, self.y*number)
        return point 

    def __mul__(self, number):
        point = Point2d(self.x*number, self.y*number)
        return point 

    def __div__(self, number):
        x = 1. * self.x / number
        y = 1. * self.y / number
        point = Point2d(x, y)

        return point 

        
    def __str__(self):
        return "x: {}, y:{}".format(round(self.x, 3), round(self.y, 3))



class Pose2d(object):
    def __init__(self, position=Point2d(), angle=0.):
        assert(isinstance(position, Point2d))
        self.position = position
        self.angle = angle

    def __add__(self, pose):
        new_pose = Pose2d(self.position + pose.position, self.angle + pose.angle)
        return new_pose

    def __sub__(self, pose):
        new_pose = Pose2d(self.position - pose.position, self.angle - pose.angle)
        return new_pose

    def __str__(self):
        return str(self.position) + "angle: {}".format(round(self.angle,3))
        
    def toPoseStamped(self):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = self.position.x 
        pose_stamped.pose.position.y = self.position.y 
        
        quat = tf.transformations.quaternion_from_euler(0, 0, self.angle)

        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1] 
        pose_stamped.pose.orientation.z = quat[2] 
        pose_stamped.pose.orientation.w = quat[3] 

        return pose_stamped

    