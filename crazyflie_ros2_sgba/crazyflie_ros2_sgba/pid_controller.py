#!/usr/bin/

import math
from enum import Enum
from dataclasses import dataclass


@dataclass
class MotorPower_t:
    m1: float = 0
    m2: float = 0
    m3: float = 0
    m4: float = 0

@dataclass
class ControlCommands_t:
  roll: float = 0
  pitch: float = 0
  yaw: float = 0
  altitude: float = 0

@dataclass
class State_t:
  roll: float = 0
  pitch: float = 0
  yaw: float = 0
  yaw_rate: float = 0
  altitude: float = 0
  vx: float = 0
  vy: float = 0

@dataclass
class GainsPID_t:
    kp_att_rp: float
    kd_att_rp: float
    kp_att_y: float
    kd_att_y: float
    kp_vel_xy: float
    kd_vel_xy: float
    kp_z: float
    kd_z: float
    ki_z: float

pastAltitudeError = 0
pastYawRateError =0
pastPitchError = 0
pastRollError = 0
pastVxError = 0
pastVyError = 0
counter = 0
altitudeIntegralError = 0



def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def init_pid_attitude_fixed_height_controller():
    global pastAltitudeError
    global pastYawRateError
    global pastPitchError
    global pastRollError
    global pastVxError
    global pastVyError
    global altitudeIntegralError

    altitudeIntegralError = 0
    pastAltitudeError = 0
    pastYawRateError =0
    pastPitchError = 0
    pastRollError = 0
    pastVxError = 0
    pastVyError = 0

def pid_attitude_fixed_height_controller(actualState: State_t, desiredState: State_t, 
                                         gainsPID: GainsPID_t,dt,motorCommands: MotorPower_t):
    controlCommands = ControlCommands_t()
    controlCommands = pid_fixed_height_controller(actualState,desiredState,gainsPID,
                                                  dt,controlCommands)
    desiredState, controlCommands = pid_attitude_controller(actualState,desiredState,gainsPID,
                                               dt,controlCommands)
    motorCommands = motor_mixing(controlCommands, motorCommands)

    return desiredState, motorCommands

def pid_velocity_fixed_height_controller(actualState: State_t, desiredState: State_t, 
                                         gainsPID: GainsPID_t,dt,motorCommands: MotorPower_t):
    global counter
   
    controlCommands = ControlCommands_t()
    
    desiredState = pid_horizontal_velocity_controller(actualState,desiredState,gainsPID,dt)

    controlCommands = pid_fixed_height_controller(actualState,desiredState,gainsPID,
                                                  dt,controlCommands)
    controlCommands = pid_attitude_controller(actualState,desiredState,gainsPID,
                                               dt,controlCommands)
    motorCommands = motor_mixing(controlCommands, motorCommands)

    return motorCommands, controlCommands.altitude, controlCommands.roll, controlCommands.pitch, controlCommands.yaw #,desiredState

def pid_fixed_height_controller(actualState: State_t, desiredState: State_t, 
                                gainsPID: GainsPID_t,dt,controlCommands: ControlCommands_t):
    global pastAltitudeError
    global altitudeIntegralError
    global counter

    counter+=1
    altitudeError = (desiredState.altitude - actualState.altitude)
 

    altitudeDerivativeError = (altitudeError - pastAltitudeError)/dt
    altitudeIntegralError += (altitudeError)*dt
    controlCommands.altitude = gainsPID.kp_z * altitudeError + gainsPID.kd_z*altitudeDerivativeError + 55.368#gainsPID.ki_z * constrain(altitudeIntegralError,-55.368/gainsPID.ki_z - 5,55.368/gainsPID.ki_z + 5)
    pastAltitudeError = altitudeError
    return controlCommands

def motor_mixing(controlCommands: ControlCommands_t, motorCommands: MotorPower_t):
    #55.368
    #controlCommands.roll = 0
    motorCommands.m1 =  controlCommands.altitude - controlCommands.roll + controlCommands.pitch + controlCommands.yaw
    motorCommands.m2 =  controlCommands.altitude - controlCommands.roll - controlCommands.pitch - controlCommands.yaw
    motorCommands.m3 =  controlCommands.altitude + controlCommands.roll - controlCommands.pitch + controlCommands.yaw
    motorCommands.m4 =  controlCommands.altitude + controlCommands.roll + controlCommands.pitch - controlCommands.yaw
    
    return motorCommands

def pid_attitude_controller(actualState: State_t, desiredState: State_t, 
                            gainsPID: GainsPID_t,dt,controlCommands: ControlCommands_t):
    global pastPitchError
    global pastRollError
    global pastYawRateError

    #Calc errors
    pitchError = desiredState.pitch - actualState.pitch
    pitchDerivativeError = (pitchError - pastPitchError)/dt
    rollError = desiredState.roll - actualState.roll
    rollDerivativeError = (rollError - pastRollError)/dt
    yawRateError = (desiredState.yaw_rate - actualState.yaw_rate)

    #PID Control
    controlCommands.roll = gainsPID.kp_att_rp * rollError + gainsPID.kd_att_rp*rollDerivativeError
    controlCommands.pitch = -gainsPID.kp_att_rp * pitchError - gainsPID.kd_att_rp*pitchDerivativeError
    controlCommands.yaw = gainsPID.kp_att_y * yawRateError

    pastPitchError = pitchError
    pastYawRateError = yawRateError
    pastRollError = rollError

    return controlCommands

def pid_horizontal_velocity_controller(actualState: State_t, desiredState: State_t, 
                                       gainsPID: GainsPID_t,dt):
    global pastVxError
    global pastVyError

    vxError = round(desiredState.vx - actualState.vx,4)
    vxDerivative = (vxError - pastVxError)/dt
    vyError = round(desiredState.vy - actualState.vy,4)
    vyDerivative = (vyError - pastVyError)/dt
    if(vxError > 4 or vyError > 4):
        vxError = 0
        vyError = 0
    pitchCommand = 10*gainsPID.kp_vel_xy * constrain(vxError,-1, 1) + gainsPID.kd_vel_xy*vxDerivative
    rollCommand = - 10*gainsPID.kp_vel_xy * constrain(vyError,-1, 1) - gainsPID.kd_vel_xy*vyDerivative
    rollCommand = -rollCommand
    desiredState.pitch = pitchCommand
    desiredState.roll = rollCommand

    pastVyError = vyError
    pastVxError = vxError

    return desiredState


    
