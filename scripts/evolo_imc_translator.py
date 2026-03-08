#!/usr/bin/python3

#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from imc_ros_bridge.msg import EstimatedState, DesiredHeading, DesiredSpeed
from std_msgs.msg import Float32
from smarc_msgs.msg import Topics as smarcTopics
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from tf_transformations import euler_from_quaternion
import math


class Translator:

    def __init__(self, node):
        #ros
        self._node = node

        self.latlon_msg = GeoPoint()
        self.odom_msg = Odometry()

        #Publishers and subscribers
        
        #Estimated state (ROS -> IMC)
        self.ros_subsciber_Odom = self._node.create_subscription(Odometry,smarcTopics.ODOM_TOPIC , self.odom_callback,10)
        self.ros_subsciber_Latlon = self._node.create_subscription(GeoPoint,smarcTopics.POS_LATLON_TOPIC, self.latlon_callback,10)
        self.ros_publisher_EstimatedState = self._node.create_publisher(EstimatedState, "imc/out/estimatedstate", 10)

        #TODO ControlLoops

        #Desired heading (IMC -> ROS)
        self.ros_subsciber_DesiredHeading = self._node.create_subscription(DesiredHeading,"imc/in/desiredheading" , self.desiredHeading_callback,10)
        self.ros_publisher_DesiredHeading_ned = self._node.create_publisher(Float32,"backseat/desiredheading" ,10)
        self.ros_publisher_DesiredHeading_enu = self._node.create_publisher(Float32,"backseat/desiredyaw" ,10)

        #Desired speed (IMC -> ROS)
        self.ros_subsciber_DesiredSpeed = self._node.create_subscription(DesiredSpeed,"imc/in/desiredspeed" , self.desiredSpeed_callback,10)
        self.ros_publisher_DesiredSpeed = self._node.create_publisher(Float32, "backseat/desiredspeed", 10)


    def odom_callback(self, msg : Odometry):
        self.odom_msg = msg

        orientation_q = self.odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        #TODO create and publish Estimated state message
        estimatedstate_msg = EstimatedState()
        estimatedstate_msg.lat = math.radians(self.latlon_msg.latitude)
        estimatedstate_msg.lon = math.radians(self.latlon_msg.longitude)
        estimatedstate_msg.height = self.odom_msg.pose.pose.position.z
        estimatedstate_msg.x = 0.0 #self.odom_msg.pose.pose.position.x
        estimatedstate_msg.y = 0.0 #self.odom_msg.pose.pose.position.y
        estimatedstate_msg.z = 0.0 #self.odom_msg.pose.pose.position.z
        estimatedstate_msg.phi = roll
        estimatedstate_msg.theta = -pitch
        estimatedstate_msg.psi = (math.pi/2.0)-yaw
        estimatedstate_msg.u = self.odom_msg.twist.twist.linear.x
        estimatedstate_msg.v = self.odom_msg.twist.twist.linear.y
        estimatedstate_msg.w = self.odom_msg.twist.twist.linear.z
        estimatedstate_msg.vx = estimatedstate_msg.u*math.cos(estimatedstate_msg.psi) + estimatedstate_msg.v*math.sin(estimatedstate_msg.psi)
        estimatedstate_msg.vy = estimatedstate_msg.u*math.sin(estimatedstate_msg.psi) + estimatedstate_msg.v*math.cos(estimatedstate_msg.psi)
        estimatedstate_msg.vz = 0.0
        estimatedstate_msg.p = self.odom_msg.twist.twist.angular.x
        estimatedstate_msg.q = -self.odom_msg.twist.twist.angular.y
        estimatedstate_msg.r = -self.odom_msg.twist.twist.angular.z
        estimatedstate_msg.depth = -self.odom_msg.pose.pose.position.z
        estimatedstate_msg.alt = self.odom_msg.pose.pose.position.z

        self.ros_publisher_EstimatedState.publish(estimatedstate_msg)

    def latlon_callback(self, msg : GeoPoint):
        self.latlon_msg = msg

    def desiredHeading_callback(self, msg: DesiredHeading):
        heaidng_msg = Float32()
        heaidng_msg.data = math.degrees(msg.value)
        self.ros_publisher_DesiredHeading_ned.publish(heaidng_msg)
        #publish yaw
        yaw_msg = Float32()
        yaw_msg.data = (math.pi / 2.0) - msg.value 
        #[0:2*pi]
        while(yaw_msg.data < 0): yaw_msg.data += 2* math.pi
        while(yaw_msg.data > 0): yaw_msg.data -= 2* math.pi
        self.ros_publisher_DesiredHeading_enu.publish(yaw_msg)
        

    def desiredSpeed_callback(self, msg : DesiredSpeed):
        speed_msg = Float32()
        if(msg.speed_units == msg.SUNITS_METERS_PS):
            speed_msg.data = msg.value
            self.ros_publisher_DesiredSpeed.publish(speed_msg)
        else:
            self._node.get_logger().error("Error unknown speed unit")

def main(args=None, namespace=None):
    rclpy.init(args=args)

    _node = Node('evolo_imc_translator')

    t = Translator(_node)
    
    while rclpy.ok():
        rclpy.spin_once(_node)

if __name__ == "__main__":
    main()



    

