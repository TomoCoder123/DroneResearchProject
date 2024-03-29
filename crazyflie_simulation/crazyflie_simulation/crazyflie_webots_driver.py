import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import cos, sin, degrees, radians, pi, isnan
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

from crazyflie_interfaces.srv import Takeoff, Land, GoTo


import os
# Change this path to your crazyflie-firmware folder
sys.path.append('/home/nicxe/CS_SITP/crazyflie-firmware')
import cffirmware

class CrazyflieWebotsDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        ## Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        self.target_twist = Twist()

        ## Initialize Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.range_front = self.robot.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = self.robot.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = self.robot.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = self.robot.getDevice("range_right")
        self.range_right.enable(timestep)

        #self.lidar = self.robot.getDevice("lidar")
        #self.lidar.enable(timestep)

        ## Intialize Variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0
        self.past_time = self.robot.getTime()

        self.first_pos = True
        self.first_x_global = 0.0
        self.first_y_global = 0.0


        cffirmware.controllerPidInit()

        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_webots_driver')
        self.node.get_logger().info(os.environ['WEBOTS_CONTROLLER_URL'])
        self.crazyflie_name = os.environ['WEBOTS_CONTROLLER_URL']

        self.node.create_subscription(Twist, self.crazyflie_name + '/cmd_vel', self.cmd_vel_callback, 1)
        self.laser_publisher = self.node.create_publisher(LaserScan, self.crazyflie_name + '/scan', 10)
        self.odom_publisher = self.node.create_publisher(Odometry,  self.crazyflie_name + '/odom', 10)
        self.node.create_service(Takeoff, self.crazyflie_name + "/takeoff", self._takeoff_callback)

        self.tfbr = TransformBroadcaster(self.node)

        self.msg_laser = LaserScan()
        self.node.create_timer(1.0/30.0, self.publish_laserscan_data)

        self.planner = cffirmware.planner()
        cffirmware.plan_init(self.planner)

    def _takeoff_callback(self, request, response):

        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.node.get_logger().info(
            f"takeoff(height={request.height} m,"
            + f"duration={duration} s,"
            + f"group_mask={request.group_mask}) {self.crazyflie_name}"
        )

        x_global = self.gps.getValues()[0]- self.first_x_global
        y_global = self.gps.getValues()[1] - self.first_y_global
        z_global = self.gps.getValues()[2]
        yaw = self.imu.getRollPitchYaw()[2]

        current_pos = np.array([x_global,y_global,z_global])
        state_pos = cffirmware.mkvec(*current_pos)

        t = self.robot.getTime()
        cffirmware.plan_takeoff(self.planner,
                state_pos, yaw, request.height,
                0.0, duration, t)

        return response


    def publish_laserscan_data(self):

        front_range = self.range_front.getValue()/1000.0

        back_range = self.range_back.getValue()/1000.0
        left_range = self.range_left.getValue()/1000.0
        right_range = self.range_right.getValue()/1000.0
        max_range = 3.49
        if front_range > max_range:
            front_range = float("inf")
        if left_range > max_range:
            left_range = float("inf")
        if right_range > max_range:
            right_range = float("inf")
        if back_range > max_range:
            back_range = float("inf")  

        self.msg_laser = LaserScan()
        self.msg_laser.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        self.msg_laser.header.frame_id = self.crazyflie_name
        self.msg_laser.range_min = 0.1
        self.msg_laser.range_max = max_range
        self.msg_laser.ranges = [back_range, left_range, front_range, right_range, back_range]
        self.msg_laser.angle_min = 0.5 * 2*pi
        self.msg_laser.angle_max =  -0.5 * 2*pi
        self.msg_laser.angle_increment = -1.0*pi/2
        self.laser_publisher.publish(self.msg_laser)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist
    
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.robot.getTime()



        dt = self.robot.getTime() - self.past_time

        if self.first_pos is True:
            self.first_x_global = self.gps.getValues()[0]
            self.first_y_global = self.gps.getValues()[1]
            self.first_pos = False

        ## Get measurements
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0]- self.first_x_global
        vx_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1] - self.first_y_global
        vy_global = (y_global - self.past_y_global)/dt
        z_global = self.gps.getValues()[2]
        vz_global = (z_global - self.past_z_global)/dt

        q_base = tf_transformations.quaternion_from_euler(0, 0, yaw)
        odom = Odometry()
        odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.crazyflie_name
        odom.pose.pose.position.x = x_global
        odom.pose.pose.position.y = y_global
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = sin(yaw / 2)
        odom.pose.pose.orientation.w = cos(yaw / 2)

        self.odom_publisher.publish(odom)

        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = self.crazyflie_name
        t_base.transform.translation.x = x_global
        t_base.transform.translation.y = y_global
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.z = sin(yaw / 2)
        t_base.transform.rotation.w = cos(yaw / 2)
        self.tfbr.sendTransform(t_base)

        ## Put measurement in state estimate
        # TODO replace these with a EKF python binding
        state = cffirmware.state_t()
        state.attitude.roll = degrees(roll)
        state.attitude.pitch = -degrees(pitch)
        state.attitude.yaw = degrees(yaw)
        state.position.x = x_global
        state.position.y = y_global
        state.position.z = z_global
        state.velocity.x = vx_global
        state.velocity.y = vy_global
        state.velocity.z = vz_global
        
        # Put gyro in sensor data
        sensors = cffirmware.sensorData_t()
        sensors.gyro.x = degrees(roll_rate)
        sensors.gyro.y = degrees(pitch_rate)
        sensors.gyro.z = degrees(yaw_rate)
        yawDesired=0

        ## Fill in Setpoints
            
        setState = cffirmware.plan_current_goal(self.planner, t)
        self.node.get_logger().info(str(t) + ' ' +str(setState.pos[2]))
        if isnan(setState.pos[2]):
            zpos = 0.0
        else:
            zpos = setState.pos[2]


        setpoint = cffirmware.setpoint_t()
        setpoint.mode.z = cffirmware.modeAbs
        setpoint.position.z = 1.0 #zpos
        setpoint.mode.yaw = cffirmware.modeVelocity
        setpoint.attitudeRate.yaw = degrees(self.target_twist.angular.z)*5
        setpoint.mode.x = cffirmware.modeVelocity
        setpoint.mode.y = cffirmware.modeVelocity
        setpoint.velocity.x = self.target_twist.linear.x
        setpoint.velocity.y = self.target_twist.linear.y
        setpoint.velocity_body = True

        ## Firmware PID bindings
        control = cffirmware.control_t()
        tick = 100 #this value makes sure that the position controller and attitude controller are always always initiated
        cffirmware.controllerPid(control, setpoint,sensors,state,tick)

        ## 
        cmd_roll = radians(control.roll)
        cmd_pitch = radians(control.pitch)
        cmd_yaw = -radians(control.yaw)
        cmd_thrust = control.thrust

        ## Motor mixing
        motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
        motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
        motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
        motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw
        
        scaling = 1000 #Todo, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1/scaling)
        self.m2_motor.setVelocity(motorPower_m2/scaling)
        self.m3_motor.setVelocity(-motorPower_m3/scaling)
        self.m4_motor.setVelocity(motorPower_m4/scaling)

        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global
        self.past_z_global = z_global
        #self.node.get_logger().info('step...')
