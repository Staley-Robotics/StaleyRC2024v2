from enum import Enum
import typing

from wpilib import DriverStation
from wpimath.geometry import *
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.units import degreesToRadians
from ntcore import *
from ntcore.util import ntproperty

from commands2 import Subsystem

class VisionConstants:
    StdDevX = ntproperty( "Settings/Vision/StdDevX", 1.0 )
    StdDevY = ntproperty( "Settings/Vision/StdDevY", 1.0 )
    StdDevR = ntproperty( "Settings/Vision/StdDevR", 1.0 )
    LockedInRange = ntproperty( "Settings/Vision/LockedInRange", 1.0 )

class Vision(Subsystem):
    def __init__(self, name:str, swerveOdometry:typing.Callable[[], SwerveDrive4PoseEstimator]):
        self.offline = ntproperty( f"Settings/Vision/{name}/Offline", False )

        self.name = name
        self.swerveOdometry = swerveOdometry
        self.useLockedInRange:bool = False

        self.wpiblue:DoubleArraySubscriber = NetworkTableInstance.getDefault().getTable( name ).getDoubleArrayTopic("botpose_wpiblue").subscribe([0.0])
        self.wpired:DoubleArraySubscriber = NetworkTableInstance.getDefault().getTable( name ).getDoubleArrayTopic("botpose_wpired").subscribe([0.0])

        self.blueQueue = []
        self.redQueue = []

    def periodic(self):
        # Offline Mode
        if self.offline:
            # Dump Queue
            self.wpiblue.readQueue()
            self.wpired.readQueue()
            return

        # Obtain Queue Data
        self.blueQueue = self.processQueue( self.wpiblue.readQueue() )
        self.redQueue = self.processQueue( self.wpired.readQueue() )

        # Publish Queue Data to 
        queue = []
        match DriverStation.getAlliance():
            case DriverStation.Alliance.kBlue:
                queue = self.blueQueue
            case DriverStation.Alliance.kRed:
                queue = self.redQueue

        for y in range(len(queue)):
            # For Vision Accuracy, if enabled
            if self.useLockedInRange:
                currentPosition = self.__getOdometry().getEstimatedPosition().translation()
                visionPosition = queue[y]['pose2d'].translation()
                distance = currentPosition.distance( visionPosition )
                if distance > VisionConstants.LockedInRange:
                    continue

            # Update Odometry
            self.__getOdometry().addVisionMeasurement(
                queue[y]['pose2d'],
                queue[y]['poseTimestamp'],
                [ VisionConstants.StdDevX, VisionConstants.StdDevY, VisionConstants.StdDevR ]
            )

    def processQueue(self, queue:list[TimestampedDoubleArray]) -> list:
        returnQueue = []
        for i in range(len(queue)):
            t = queue[i].time
            tx = queue[i].value[0]
            ty = queue[i].value[1]
            tz = queue[i].value[2]
            rr = degreesToRadians( queue[i].value[3] )
            rp = degreesToRadians( queue[i].value[4] )
            ry = degreesToRadians( queue[i].value[5] )
            d = queue[i].value[9]
            l = queue[i].value[6]

            # Data Not Found
            if tx == 0.0 and ty == 0.0 and ry == 0.0:
                continue
            # TX Data Off Field
            if tx < 0.0 or tx > 16.523:
                continue
            # TY Data Off Field
            if ty < 0.0 or ty > 8.013:
                continue

            returnQueue.append(
                {
                    "poseTimestamp": (t / 1000000.0) - ( l / 1000 ), 
                    "distance": d,
                    "latency": l / 1000,
                    "pose2d": Pose2d( tx, ty, Rotation2d( ry ) ),
                    "pose3d": Pose3d( Translation3d( tx, ty, tz ), Rotation3d( rr, rp, ry ) )
                }
            )
        
        return returnQueue

    def __getOdometry(self) -> SwerveDrive4PoseEstimator:
        return self.swerveOdometry()
    
    def toggleUseLockedInRange(self) -> None:
        self.useLockedInRange = not self.useLockedInRange

    def setUseLockedInRange(self, value:bool) -> None:
        self.useLockedInRange = value
    

