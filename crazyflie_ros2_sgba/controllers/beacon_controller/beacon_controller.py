import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import cos, sin, fabs, pi,sqrt, atan2, atan

import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import cffirmware
import pid_controller
import wallFollower

# create the Robot instance.

class beaconDriver:




    def init(self, webots_node: Node, properties):
        self.props = (properties)
        self.manual = False
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        self.emitter = self.robot.getDevice("emitter")
        self.emitter.bufferSize = 32
        self.emitter.baudRate = 2000000
        self.emitter.channel = 1

        

        rclpy.init(args=None)

        self.node = rclpy.create_node('beacon_controller')
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        print = self.node.get_logger().info

        self.emitter.send(32)
        print("signal sent")

    
    

# Enter here exit cleanup code.
