#!/usr/bin/python3
from enum import Enum
from dataclasses import dataclass
import math



@dataclass
class CommandVel_t:
    cmdVelX: float
    cmdVelY: float
    cmdVelZ: float 

class wallFollower:
    def __init__(self):
        self.refDistanceFromWall = 0.0
        self.maxForwardSpeed = 0.2
        self.maxTurnRate = 0.5
        self.direction = 1.0
        self.firstRun = False
        self.prevHeading = 0.0
        self.wallAngle = 0.0
        self.aroundCornerBackTrack = False
        self.stateStartTime = 0
        self.rangerValueBuffer = 0.2
        self.angleValueBuffer = 0.01
        self.rangerThresholdLost = 0.3
        self.inCornerAngle = 0.8
        self.waitForMeasurementSeconds = 1.0

        self.stateWF = Enum('stateWF', ['forward',
                                        'hover',
                                        'turnToFindWall',
                                        'turnToAlignToWall',
                                        'forwardAlongWall',
                                        'rotateAroundWall',
                                        'rotateInCorner',
                                        'findCorner'])
        

        self.timeNow = 0

    def wallFollowerInit(self, refDistanceFromWallNew, maxForwardSpeed_ref):
        self.refDistanceFromWall = refDistanceFromWallNew
        self.maxForwardSpeed = maxForwardSpeed_ref
        self.firstRun = True
    
    def isCloseToo(real_value: float, checked_value: float, margin: float):
        return (real_value > checked_value - margin and real_value < checked_value + margin)

    def wraptopi(num: float)->float:
        if(num>math.pi):
            return num - 2*math.pi
        
        elif (num < -1*math.pi):
            return num + 2*math.pi
        
        else:
            return num
        
    def commandTurn(self, cmdVelX: float, cmdAngW: float, ref_rate: float):
        cmdVelX = 0
        cmdAngW = self.direction * ref_rate
        return cmdVelX, cmdAngW
    
    def commandAlignCorner(self, cmdVelY: float,cmdAngW: float, ref_rate: float, range: float, wanted_distance_from_corner: float):
        if(range > wanted_distance_from_corner + 0.3):
            cmdAngW = self.direction * ref_rate
            cmdVelY = 0
        else:
            if (range > wanted_distance_from_corner):
                cmdVelY = self.direction * -(self.maxForwardSpeed/3)
            else:
                cmdVelY = self.direction *  (self.maxForwardSpeed/3)
        cmdAngW = 0
        return cmdVelY, cmdAngW
    
    def commandHover(cmdVelX: float, cmdVelY: float,cmdAngW: float):
        cmdVelX = 0
        cmdAngW = 0
        cmdVelY = 0

        return cmdVelX,cmdVelY,cmdAngW
    
    def commandFowardAlongWall(self, cmdVelX: float,cmdVelY: float,range: float):

        cmdVelX = self.maxForwardSpeed
        check_distance_wall = self.isCloseToo(self.refDistanceFromWall, range,0.1)
        cmdVelY = 0

        if(not check_distance_wall):
            if(range > self.refDistanceFromWall):
                cmdVelY = self.direction * -(self.maxForwardSpeed/2)
            else:
                cmdVelY = self.direction * (self.maxForwardSpeed/2)

        return cmdVelX, cmdVelY
    
    def commandTurnAndAdjust(self, cmdVelY: float, cmdAngW: float,rate: float, range: float):
        cmdAngW = self.direction * rate
        cmdVelY = 0

        return cmdVelY, cmdAngW
    
    def transition(self, newState: Enum):
        self.stateStartTime = self.timeNow
        return newState
    
    def adjustDistanceWall(self, distanceWallNew: float):
        self.refDistanceFromWall = distanceWallNew

    def wallFollower(self,commandVel: float,rontRange: float, sideRange: float,currentHeading: float,
                    directionTurn: int, timeOuter: float):
        self.direction = directionTurn

        self.timeNow = timeOuter

        if self.firstRun:
            self.prevHeading = currentHeading
            self.aroundCornerBackTrack = False
            self.firstRun = False

        if(self.stateWF == self.stateWF.forward):
            if(self.frontRange < self.refDistanceFromWall + self.rangerValueBuffer):
                self.stateWF = self.transition(self.stateWF.turnToFindWall)
        
                cmdVelXTemp, cmdVelYTemp = self.commandTurn(fsa,fsaf,fsa)
        pass

