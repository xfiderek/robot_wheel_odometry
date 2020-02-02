import rospy 

from std_msgs.msg import String
from std_srvs.srv import SetBool

from nav_msgs.msg import Odometry 
from robot_wheel_odometry.msg import WheelShifts

from chassis import Chassis
from odom_calculator import OdometryCalculator

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('wheel_odom_publisher')
        self.chassis = Chassis()
        self.wheel_shifts_sub = rospy.Subscriber('/wheel_shifts', WheelShifts, self.publishOdom)        
        self.odometry_pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=1)
        self.odometry_calc = OdometryCalculator(self.chassis)

        self.rate = rospy.Rate(15)
        self.reset_odom_service = rospy.Service('reset_wheel_odom', SetBool, self.resetWheelOdom) #resets odometry to 0,0,0

    def publishOdom(self, wheel_shifts):
        self.chassis.updateState(wheel_shifts)
        odom = self.odometry_calc.calcOdom()
        self.odometry_pub.publish(odom)
        self.rate.sleep()

    def resetWheelOdom(self, req):
        if req.data == True:
            self.odometry_calc.resetPose()
        
        return True, "ok"