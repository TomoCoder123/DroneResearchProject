#!/usr/bin/python3

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


import struct

#DEPRACATED
#import cffirmware

import pid_controller
import wallFollower


class beaconDriver:
    def init(self, webots_node: Node, properties):
        rclpy.init(args=None)

        self.node = rclpy.create_node('beacon_controller')
        self.props = (properties)
        self.manual = False
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        self.emitter = self.robot.getDevice("emitter")
        self.emitter.bufferSize = 32
        self.emitter.baudRate = 2000000
        self.emitter.setChannel(1)
        

        
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        print = self.node.get_logger().info
        message =  '0'
        if(int(self.robot.getTime()*10) % 2 == 0):
           self.emitter.send(message.encode())

        
        # print("signal sent")
        
class sgbaDriver:
    timesp = 0
    first_run = False
    ref_distance_from_wall = 0
    max_speed = 0.5
    priority = False
    rssi_threshold = 1500 #need to change based on converted values
    rssi_collision_threshold = 1100

    wanted_angle = 0
    own_id = 0
    outbound = True
    state_wf = 1
    near_drone_id = 0
    state_start_time: int
    
    state = 2
    wanted_angle_dir = 0 #wanted angle - current heading
    pos_x_hit = 0 
    pos_y_hit = 0
    pos_x_sample = 0 
    pos_y_sample = 0
    overwrite_and_reverse_direction = False
    direction = 1
    cannot_go_to_goal = False
    prev_rssi = 150
    diff_rssi = 0
    rssi_sample_reset = False
    heading_rssi = 0
    correct_heading_array = [0] * 8
    first_time_inbound = True
    wanted_angle_hit = 0
    rssi_inter = 0
    rssi_inter_angle = 0

    decodedSignal = 0
#1 forward
#2 rotate to goal
#3 wall following
#4 move out of way

    def init(self, webots_node: Node, properties):
        
        self.props = (properties)
        self.manual = False
        self.robot = webots_node.robot
        self.set_id()

        timestep = int(self.robot.getBasicTimeStep())
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(timestep)
        self.receiver.setChannel(1)
        #self.receiver.signalStrengthNoise = 0.2
        #2Mbps, 1Mbps and 250Kps communication data-rate
        #Sends and receives data packets of up to 32 bytes payload
        self.emitter = self.robot.getDevice("emitter")
        self.emitter.bufferSize = 32
        self.emitter.baudRate = 2000000
        self.emitter.setChannel(1)
        

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

        self.wallFollowerO = wallFollower.WallFollowing()
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

        self.rssi_angle = 0
    

        #init self.state vars for PID
        self.actualState = pid_controller.State_t()
        self.desiredState = pid_controller.State_t()
        self.motorPower = pid_controller.MotorPower_t()

        #init PID values 
        self.gainsPID = pid_controller.GainsPID_t(0.5,0.1,1,1,0.5,1,5,5,0)
        pid_controller.init_pid_attitude_fixed_height_controller()

        ## Intialize Variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0
        self.past_time = self.robot.getTime()

        self.cmd_vel_x = 0
        self.cmd_vel_y = 0
        self.cmd_vel_w = 0


        self.first_pos = True
        self.first_x_global = 0.0
        self.first_y_global = 0.0

        #cffirmware.controllerPidInit()

        rclpy.init(args=None)
        self.node = rclpy.create_node('sgba_controller')
        self.laser_publisher = self.node.create_publisher(LaserScan, 'scan', 10)
        self.odom_publisher = self.node.create_publisher(Odometry, 'odom', 10)
        self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        self.tfbr = TransformBroadcaster(self.node)

        self.msg_laser = LaserScan()
        self.node.create_timer(1.0/30.0, self.publish_laserscan_data)
        #supports up to 9 drones
        if (self.own_id == 1):
            
            self.init_gradient_bug_loop_controller(0.4,0.5,0.8,0)
        elif (self.own_id == 2):
            self.init_gradient_bug_loop_controller(0.4, 0.5,-0.8,0)


        self.beacon_rssi = 0
    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        dt = self.robot.getTime() - self.past_time
        print = self.node.get_logger().info
        #print(str(self.props))

        print("---------------------STEP---------------------")
        #print("id: " + str(self.own_id))
        
        imu = self.imu.getRollPitchYaw()
        self.actualState.roll = imu[0]
        self.actualState.pitch = imu[1]
        self.actualState.yaw = imu[2]

        gps = self.gps.getValues()
        self.actualState.yaw_rate = self.gyro.getValues()[2]
        self.actualState.altitude = gps[2]

        globalX = gps[0]
        globalY = gps[1]
        #red is x green is y
        #print("x " + str(globalX)) #up down y  left right x
        globalVX = (globalX - self.past_x_global)/dt
        globalVY = (globalY - self.past_y_global)/dt

        actualYaw = imu[2]
        print("ACtual Yaw: " + str(actualYaw))

        local_angle = actualYaw - self.wraptopi(atan2(globalVY,globalVX))
        magnitude = sqrt(globalVX*globalVX + globalVY*globalVY)

        localVY = sin(local_angle) * magnitude
        localVX = (cos(local_angle) * magnitude)

        print("VX,VY: (" + str(round(localVX,10)) + ", " + str(round(localVY,10)) + ")")
        #cosyaw = cos(actualYaw)
        #sinyaw = sin(actualYaw)
        
        self.actualState.vx = localVX#globalVX * cosyaw + globalVY*sinyaw
        self.actualState.vy = localVY#globalVY * sinyaw + globalVY*cosyaw
        if(self.get_time()<5):
            self.desiredState = pid_controller.State_t(0,0,0,0,0.7,0,0)
        else:
            #self.desiredState = pid_controller.State_t(0,0.1,0,1,1,0,0)
            pass


        range_front_value = self.range_front.getValue()/1000
        range_left_value = self.range_left.getValue()/1000
        range_right_value = self.range_right.getValue()/1000
        range_back_value = self.range_back.getValue()/1000
        print("front: " + str(range_front_value) + " left: " + str(range_left_value) + " right: "+ str(range_right_value) + " back " + str(range_back_value))
        
        signal = 0
        #print("timestep"+ str(self.robot.getTime()))
        
        if(int(self.robot.getTime()*10) % 2 == 0):
            message =  self.robot.getName()[9]
            self.emitter.send(message.encode())
            #print("sent signal at modulus2")
        #print("QueueLength: "+ str(self.receiver.getQueueLength()) )
            
        while(self.receiver.getQueueLength()>0):
            
            self.decodedSignal = int(self.receiver.getData().decode())
            #print("decodedSignal: " + str(decodedSignal))

            if(int(self.decodedSignal > 0 )):
                
                    #print("decodedSignal: " + str(decodedSignal) +" signal received from drone " + str(self.receiver.getData()))
                    direction = self.receiver.getEmitterDirection()
                    self.rssi_inter_angle = self.wraptopi(atan2(direction[2], direction[0]))
                    self.rssi_inter = 100*(self.receiver.getSignalStrength())
                    #print("angle: "+ str(self.rssi_inter_angle))
                    signal = self.rssi_inter
                    print("decodedSignal: " + str(self.decodedSignal))
                    print("signal before if statement: " + str(signal))
                    if(signal > self.rssi_collision_threshold and signal < self.rssi_threshold): 
                        self.setNearDroneId(self.decodedSignal) 
                        print("near drone id: " + str(self.near_drone_id))
                        self.set_priority()
                        print("priorty: " + str(self.priority))


            else:
                    self.beacon_rssi = 100 * self.receiver.getSignalStrength()
                    #self.beacon_rssi = self.ss2rssi(signal)
                    signal = self.beacon_rssi
                #print(signal)
        

        
            self.receiver.nextPacket()



        print("inter signal:" + str(self.rssi_inter) + "beacon rssi: " + str(self.beacon_rssi))


        print("---------GB_LOOP_CONTROLLER----------")
        if(self.get_time()>6 and not self.manual):
            self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_w = self.gradient_bug_loop_controller(self.cmd_vel_x,self.cmd_vel_y,self.cmd_vel_w,self.rssi_angle,self.state_wf,
                                                                     range_front_value,range_left_value,range_right_value,range_back_value,
                                                                     actualYaw,globalX,globalY,self.beacon_rssi,self.rssi_inter,self.rssi_inter_angle,self.priority,self.outbound)
        print("-------------------------------------")

        if(self.manual):
            self.cmd_vel_x = self.target_twist.linear.x
            self.cmd_vel_y = self.target_twist.linear.y
            self.cmd_vel_w = self.target_twist.angular.z

        self.desiredState.yaw_rate = self.cmd_vel_w
        self.desiredState.vx = self.cmd_vel_x
        self.desiredState.vy = self.cmd_vel_y
            
        #print(str(self.cmd_vel_w))

        #self.node.get_logger().error("counter: "+str(pid_controller.counter)+"- "+str(pid_controller.pastYawRateError) + " +=" + str(self.desiredState.yaw_rate - self.actualState.yaw_rate) + "/ dt:" + str(dt) + ": "+ str(self.motorPower))
        #print(str(round(self.desiredState.roll,9))+ ": "  + str(""))


        # if(pid_controller.counter%10 == 0):
        #     print(str(self.actualState))
        #     print(str(self.desiredState))
        #     print(str(self.motorPower))
        #     print(str(pid_controller.counter))

        
        # motorPowers = self.pid.pid(dt,self.desiredState.vx,self.desiredState.vy,self.desiredState.yaw_rate,
        #              self.desiredState.altitude,self.actualState.roll,self.actualState.pitch
        #              ,self.actualState.yaw_rate,self.actualState.altitude,self.actualState.vx
        #              ,self.actualState.vy)
        #self.desiredState, 
        junk,alt,roll,pitch,yaw  = pid_controller.pid_velocity_fixed_height_controller(self.actualState,self.desiredState,self.gainsPID,dt,self.motorPower)
        # print(str(self.desiredState))
        roll = round(roll,3)
        pitch = round(pitch,3)
        yaw = round(yaw,3)
        '''print("r+p+y = "+str(roll)+ " + " + str(pitch) + " + " + str(yaw) + 
              "| Roll Error:(" + str(round(self.desiredState.roll,4)) + " - " + str(round(self.actualState.roll,4))+")"+ 
              "| Yaw_R Error:(" + str(round(self.desiredState.yaw_rate,4)) + "- " + str(round(self.actualState.yaw_rate,4))+")")*#
        #print(str(self.motorPower))'''

        self.m1_motor.setVelocity(-self.motorPower.m1)
        self.m2_motor.setVelocity( self.motorPower.m2)
        self.m3_motor.setVelocity(-self.motorPower.m3)
        self.m4_motor.setVelocity( self.motorPower.m4)


        self.past_time = self.robot.getTime()
        self.past_x_global = globalX
        self.past_y_global = globalY


    def set_id(self):
        self.own_id = int(self.robot.getName()[9])

    def deg2rad(angleDegrees):
        return (angleDegrees * pi / 180)

    def rad2deg(self, angleRadians)->float:
        return (angleRadians * 180 / pi)
    
    def wraptopi(self, num):
        if(num>pi):
            return num - (2*pi)
        elif num < -pi:
            return num + 2*pi
        else:
            return num
    
    def ss2rssi(self, signal_strength):
        rssi = sqrt(1/signal_strength)
        return rssi
    
    def set_priority(self):
        if(self.near_drone_id > self.own_id):
            self.priority = False
        else:
            self.priority = True
    def setNearDroneId(self, message):
        
        self.near_drone_id = message


    def maxValue(self, arr, size):
        maxValue = arr[0]

        for i in range(size):
            if(arr[i]>maxValue):
                maxValue = arr[i]
        
        return maxValue
    
    def transition(self,new_state: int)->int:
        self.state_start_time = self.get_time()
        #self.node.get_logger().info("chaning state to (" + str(self.state_wf) + ")")
        return new_state
    
    def get_time(self):
        return self.robot.getTime()
    
    def isCloseToo(self, real_value, checked_value, margin):
        return (real_value > checked_value - margin and real_value < checked_value + margin)
            
    def fillHeadingArray(self, correct_heading_array, rssi_heading, diff_rssi, max_meters):
        heading_arr = [-135, -90, -45, 0, 45, 90, 135, 180]
        rssi_heading_deg = self.rad2deg(rssi_heading)
        for it in range(8):
            if((rssi_heading_deg >= heading_arr[it] - 22.5 and rssi_heading_deg < heading_arr[it] + 22. and it != 7) or (
                it == 7 and (rssi_heading_deg >= heading_arr[it] - 22.5 or rssi_heading_deg < -135.0 - 22.5))):
                
                temp_value_forward = correct_heading_array[it]
                temp_value_back = correct_heading_array[(it+4)%8]
            if(diff_rssi > 0):
                correct_heading_array[it] = temp_value_forward + 1
                if (temp_value_back > 0):
                    correct_heading_array[(it + 4) % 8] = temp_value_back - 1
            
            elif(diff_rssi<0):
                correct_heading_array[it] = temp_value_forward - 1
                if (temp_value_back > 0):
                    correct_heading_array[(it + 4) % 8] = temp_value_back + 1
        
        if(self.maxValue(correct_heading_array,8) > max_meters):
            for it in range(8):
                if(correct_heading_array[it]> 0):
                    correct_heading_array[it] -= 1
        
        count = 0
        y_part, x_part = 0
        for it in range(8):
            if(correct_heading_array[it]>0):
                x_part += correct_heading_array[it]* cos(heading_arr[it]*pi/180)
                y_part += correct_heading_array[it]* sin(heading_arr[it]*pi/180)

                count += correct_heading_array[it]

        wanted_angle_return = 0
        if(count != 0):
            wanted_angle_return = atan2(y_part/ count, x_part/count)

        return wanted_angle_return
    
    def commandTurn(self, max_rate):
        return max_rate

    def init_gradient_bug_loop_controller(self, new_ref_distance_from_wall,  max_speed_ref,
                                           begin_wanted_heading,initState):
        self.ref_distance_from_wall = new_ref_distance_from_wall
        self.max_speed = max_speed_ref
        self.wanted_angle = begin_wanted_heading
        self.first_run = True

    
    def gradient_bug_loop_controller(self, vel_x,  vel_y, vel_w,  rssi_angle, state_wallfollowing,
                                  front_range,  left_range,  right_range,  back_range,
                                  current_heading,  current_pos_x,  current_pos_y,  rssi_beacon,
                                  rssi_inter,  rssi_angle_inter, priority, outbound):
        print = self.node.get_logger().info
        print("transition State_Wallfollowing: " + str(self.state_wf))
        if (self.first_run):
            self.wanted_angle_dir = self.wraptopi(current_heading - self.wanted_angle)
            self.state = 2
            self.state_start_time = self.get_time()
            self.first_run = False
            self.wallFollowerO.reference_distance_from_wall = 0.25
            self.wallFollowerO.max_forward_speed = 0.3

        if (self.first_time_inbound):
            self.wraptopi(self.wanted_angle - pi)
            self.wanted_angle_dir = self.wraptopi(current_heading - self.wanted_angle)
            self.state = self.transition(2)
            self.first_time_inbound = False
        
        #***********************************************************
        # Start of Transition State Handling
        #***********************************************************
        
        if (self.state == 1):
            if(self.overwrite_and_reverse_direction):
                self.direction = -1 * self.direction
                self.overwrite_and_reverse_direction = False
            else:
                if (left_range < right_range and left_range < 2): #if wall is close to left range
                    self.direction = -1
                elif (left_range > right_range and  right_range < 2): #if wall is close to right range
                    self.direction = 1
                elif(left_range > 2 and right_range >2):
                    self.direction = 1

            self.pos_x_hit = current_pos_x #when meets wall
            self.pos_y_hit = current_pos_y
            self.wanted_angle_hit = self.wanted_angle #angle at which wall was hit
            self.wallFollowerO.reference_distance_from_wall = 0.25 #distance from wall threshold
            self.wallFollowerO.max_forward_speed = 0.3 
            for i in range (8):
                self.correct_heading_array[i] = 0 #reset heading array
            self.state = self.transition(3) #transition to wall following
        elif (self.state == 2): #hover
            goal_check = self.isCloseToo(self.wraptopi(current_heading - self.wanted_angle),0,0.1)
            #print("angle to goal:" + str(self.wraptopi(current_heading - self.wanted_angle)) + "< 0.1: " + str(goal_check))
            if (front_range < self.ref_distance_from_wall + 0.2): #if there is a wall in front
                self.cannot_go_to_goal = True

                self.state = self.transition(3) #transition to turn to find wall
            if goal_check: #transition to forward
                self.state = self.transition(1)


        elif (self.state == 3): #turn to find wall
            # self.timesp += 1
            # if (self.timesp>1200):
            #     outbound = False
            
            if(self.priority == False and rssi_inter < self.rssi_threshold):
                if (outbound):
                    if(rssi_angle_inter < 0 and self.wanted_angle < 0 or rssi_angle_inter > 0 and self.wanted_angle >0):
                        self.wanted_angle = -1 * self.wanted_angle
                        self.wanted_angle_dir = self.wraptopi(current_heading - self.wanted_angle)
                
                if (rssi_inter > self.rssi_collision_threshold and self.get_time() > 7):
                    print("initiate transitonnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn to move out of way state")
                    self.state = self.transition(4)
            if (self.state_wf == 5 and self.cannot_go_to_goal):
                self.cannot_go_to_goal = False
            bearing_to_goal = self.wraptopi(self.wanted_angle - current_heading)
            goal_check_WF = False
            if (self.direction == -1):
                goal_check_WF = bearing_to_goal < 0 and bearing_to_goal > 1.57
            else:
                goal_check_WF = bearing_to_goal > 0 and bearing_to_goal < 1.57

            rel_x_loop = current_pos_x - self.pos_x_hit
            rel_y_loop = current_pos_y - self.pos_y_hit
            loop_angle = self.wraptopi(atan2(rel_y_loop, rel_x_loop))
            if (abs(self.wraptopi(self.wanted_angle_hit + pi - loop_angle)<1)):
                self.overwrite_and_reverse_direction = True
            frontrangecheck = front_range > self.ref_distance_from_wall + 0.4
            if ((self.state_wf == 6 or self.state_wf == 7) and goal_check_WF and front_range > self.ref_distance_from_wall + 0.4 and not self.cannot_go_to_goal):
                    self.wanted_angle_dir = self.wraptopi(current_heading - self.wanted_angle)
                    self.state = self.transition(2)
            if (self.state_wf ==5):
                    if(not outbound):
                        if(self.rssi_sample_reset):
                            self.pos_x_sample = current_pos_x
                            self.pos_y_sample = current_pos_y
                            self.rssi_sample_reset
                            self.prev_rssi = rssi_beacon
                        rel_x_sample = current_pos_x - self.pos_x_sample
                        rel_y_sample = current_pos_y - self.pos_y_sample
                        distance = sqrt(rel_x_sample * rel_x_sample + rel_y_sample * rel_y_sample)
                        if (distance > 1):
                            #only fill heading array on inbound
                            self.rssi_sample_reset = True
                            self.heading_rssi = current_heading
                            diff_rssi_unf = self.prev_rssi - rssi_beacon
                            self.diff_rssi = diff_rssi_unf
                            self.wanted_angle = self.fillHeadingArray(self.correct_heading_array, self.heading_rssi, self.diff_rssi, 4)
                    else:
                        self.rssi_sample_reset = True
        elif(self.state == 4):
            if(rssi_inter <= self.rssi_collision_threshold):
                self.state = self.transition(2)
        # if(rssi_inter >= self.rssi_collision_threshold):
        #     self.state = self.transition(2)
        temp_vel_x = 0
        temp_vel_y = 0
        temp_vel_w = 0 

        #***********************************************************
        # Start of Action State Handling
        #***********************************************************

        print("Action State is (" + str(self.state) + ")")
        if (self.state == 1):
            if(left_range < self.ref_distance_from_wall):
                temp_vel_y = -0.2
            if (right_range < self.ref_distance_from_wall):
                temp_vel_y = 0.2
            temp_vel_x = 0.5
        elif(self.state == 2):
           print("wanted_angle_dir: "+str(self.wanted_angle_dir))
           if (self.wanted_angle_dir < 0):
                temp_vel_w = self.commandTurn(0.5) #whydoes changing commandTurn magnitude make it not crash 0.5: 1 crash 2 doesn't 0.05 1 gets stuck 0.1, 1 crashes
           else:
                temp_vel_w = self.commandTurn(-0.5) #is -0.5
           print("temp_vel_w: " + str(temp_vel_w))
            
        elif(self.state == 3):
            if(self.direction == -1):
            #might be jank

                temp_vel_x, temp_vel_y, temp_vel_w, self.state_wf  = self.wallFollowerO.wall_follower(front_range, left_range, current_heading,self.direction, self.get_time())
                print("command_vel_x: "+ str(temp_vel_x)+ "command vel y temp: " + str(temp_vel_y)+ " command angle rate (w) temp: "+ str(temp_vel_w))
                
            else: 
                temp_vel_x, temp_vel_y, temp_vel_w, self.state_wf = self.wallFollowerO.wall_follower( front_range, right_range, current_heading, self.direction, self.get_time())
                                                                                          
                print("command_vel_x: "+ str(temp_vel_x)+ "command vel y temp: " + str(temp_vel_y)+ " command angle rate (w) temp: "+ str(temp_vel_w))


        elif(self.state == 4):
            save_distance = 0.7
            print("left_range: " + str(left_range) + " right_range: "+ str(right_range)+ " front_range: "  + str(front_range) + " back_range: " + str(back_range))
            if (left_range < save_distance): #save distance??
                temp_vel_y = temp_vel_y + 0.35
            if (right_range < save_distance):
                temp_vel_y = temp_vel_y - 0.35
        
            if (front_range < save_distance):
                temp_vel_x = temp_vel_x + 0.35
        
            if (back_range < save_distance):
                temp_vel_x = temp_vel_x - 0.35
        
        print("tempvel x: " + str(temp_vel_x) + "tempvel y: " + str(temp_vel_y) + "temp_vel_w: " + str(temp_vel_w))
        print("(XV,YV, YawRate) from WF: " + "("+str(self.cmd_vel_x)+", "+str(self.cmd_vel_y)+", "+str(self.cmd_vel_w)+ ")")

        self.rssi_angle = self.wanted_angle

        vel_x = temp_vel_x
        vel_y = temp_vel_y
        vel_w = temp_vel_w
        return temp_vel_x, temp_vel_y, temp_vel_w
    
    def publish_laserscan_data(self):

        front_range = float(self.range_front.getValue()/1000.0)

        back_range = float(self.range_back.getValue()/1000.0)
        left_range = float(self.range_left.getValue()/1000.0)
        right_range = float(self.range_right.getValue()/1000.0)
        max_range = 3.49
        if front_range > max_range:
            front_range = 0.0
        if left_range > max_range:
            left_range = 0.0
        if right_range > max_range:
            right_range = 0.0
        if back_range > max_range:
            back_range = 0.0

        self.msg_laser = LaserScan()
        self.msg_laser.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        self.msg_laser.header.frame_id = 'base_link'
        self.msg_laser.range_min = 0.1
        self.msg_laser.range_max = max_range
        #print('print',back_range, left_range, front_range, right_range, back_range)
        self.msg_laser.ranges = [back_range, left_range, front_range, right_range, back_range]
        #self.msg_laser.ranges = [max_range, max_range, max_range, max_range, max_range]

        self.msg_laser.angle_min = 0.5 * 2*pi
        self.msg_laser.angle_max =  -0.5 * 2*pi
        self.msg_laser.angle_increment = -1.0*pi/2
        self.laser_publisher.publish(self.msg_laser)

    def publish_odom_data(self, x_global,y_global,yaw):
        q_base = tf_transformations.quaternion_from_euler(0, 0, yaw)
        odom = Odometry()
        odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x_global
        odom.pose.pose.position.y = y_global
        odom.pose.pose.position.z = 0.0

        #odom.pose.pose.orientation.x = q_base[0]
        #odom.pose.pose.orientation.y = q_base[1]
        #odom.pose.pose.orientation.z = q_base[2]
        #odom.pose.pose.orientation.w = q_base[3]
        odom.pose.pose.orientation.z = sin(yaw / 2)
        odom.pose.pose.orientation.w = cos(yaw / 2)

        self.odom_publisher.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     sd = self()
#     rclpy.spin(sd)

#     rclpy.shutdown()
#     pass

# if __name__ == '__main__':
#     main()