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
        self.__offline = ntproperty( f"Settings/Vision/{name}/Offline", False )

        self.__getOdometryFromSwerve = swerveOdometry
        self.__useLockedInRange:bool = False

        self.__wpiblue:DoubleArraySubscriber = NetworkTableInstance.getDefault().getTable( name ).getDoubleArrayTopic("botpose_wpiblue").subscribe([0.0])
        self.__wpired:DoubleArraySubscriber = NetworkTableInstance.getDefault().getTable( name ).getDoubleArrayTopic("botpose_wpired").subscribe([0.0])

    def periodic(self):
        # Offline Mode
        if self.__offline:
            # Dump Queue
            self.__wpiblue.readQueue()
            self.__wpired.readQueue()
            return

        # Obtain Queue Data
        blueQueue = self.processQueue( self.__wpiblue.readQueue() )
        redQueue = self.processQueue( self.__wpired.readQueue() )

        # Publish Queue Data to 
        queue = []
        match DriverStation.getAlliance():
            case DriverStation.Alliance.kBlue:
                queue = blueQueue
            case DriverStation.Alliance.kRed:
                queue = redQueue

        for y in range(len(queue)):
            try:
                # For Vision Accuracy, if enabled
                if self.__useLockedInRange:
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
            except:
                # Odometry Lock in Place Ignore Command
                pass

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
        return self.__getOdometryFromSwerve()
    
    def toggleUseLockedInRange(self) -> None:
        self.__useLockedInRange = not self.__useLockedInRange

    def setUseLockedInRange(self, value:bool) -> None:
        self.__useLockedInRange = value

    def getUseLockedInRange(self) -> bool:
        return self.__useLockedInRange
    

