from wpilib import RobotState, getTime, DriverStation, Field2d, SmartDashboard

from wpimath.geometry import Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds, SwerveModulePosition, SwerveModuleState
from wpimath.estimator import SwerveDrive4PoseEstimator

from commands2 import Subsystem

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

from .SwerveModule import SwerveModule
from .CustomPigeon2 import CustomPigeon2
from .Vision import Vision

class SwerveDrive(Subsystem):
    # Variable Declaration
    m_system_id:int = None
    m_logging:NetworkTable = None

    k_maxSpeed:float = 3.0 # meters / sec

    def __init__(self, ssID:int, modules:list[SwerveModule], gyro:CustomPigeon2) -> None:
        ##Subsystem Inits
        self.setName("SwerveDrive")

        ##Logging Inits
        self.m_system_id = ssID
        self.m_value = 0.0
        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/SwerveDrive")
        self.poseTopic = self.m_logging.getStructTopic("robot pose vision", Pose2d).publish()
        self.odometryTopic = self.m_logging.getStructTopic("robot pose odometry", Pose2d).publish()

        self.vision_enable = ntproperty('Vision Enabled', True, persistent=True)

        ##Swerve Inits
        #SwerveModule offsets from center, in meters
        fl_offset = Translation2d(0.2667,  0.2667)
        fr_offset = Translation2d(0.2667, -0.2667)
        bl_offset = Translation2d(-0.2667,  0.2667)
        br_offset = Translation2d(-0.2667, -0.2667)

        #Vision
        self.vision = Vision()

        #physical components
        self.modules = modules
        self.gyro = gyro

        #converts between chassis velocity and Swerve Module States
        self.kinematics = SwerveDrive4Kinematics(
            fl_offset,
            fr_offset,
            bl_offset,
            br_offset
        )

        #tracks field position, args are the assumed starting state of the robot
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.get_rotation_2d(),
            self.get_module_positions()
        )
        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.get_rotation_2d(),
            self.get_module_positions(),
            Pose2d(0,0,self.gyro.get_rotation_2d()),
            (1.0,1.0,1.0),
            (0,0,0)
            #can include StdDevs for Pose
            #can include StdDevs for Vision
        )
        self.first_vision_recieved = False

        self.field = Field2d()
        SmartDashboard.putData('hi', self.field)

        ## Pathplanner Setup
        #load robot config from pathplanner
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getChassisSpeeds,
            lambda speeds, feedforwards: self.drive_from_chassis_speeds(speeds),
            PPHolonomicDriveController(
                PIDConstants(5.0,0.0,0.0), #Translation Pid
                PIDConstants(5.0,0.0,0.0)  #Rotation Pid
            ),
            config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            self
        )

    def updateLogging(self) -> None:
        self.m_logging.putNumber( "Gyro value", self.gyro.get_rotation_2d().degrees())
        self.poseTopic.set(self.pose_estimator.getEstimatedPosition())
        self.odometryTopic.set(self.odometry.getPose())
        
        self.m_logging.putNumber( f"SwerveModule0 velocity measured", self.modules[0].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule0 angle measured", self.modules[0].getAbsoluteEncoderPosition() )
        self.m_logging.putNumber( f"SwerveModule1 velocity measured", self.modules[1].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule1 angle measured", self.modules[1].getAbsoluteEncoderPosition() )
        self.m_logging.putNumber( f"SwerveModule2 velocity measured", self.modules[2].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule2 angle measured", self.modules[2].getAbsoluteEncoderPosition() )
        self.m_logging.putNumber( f"SwerveModule3 velocity measured", self.modules[3].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule3 angle measured", self.modules[3].getAbsoluteEncoderPosition() )

    def periodic(self) -> None:
        self.updateLogging()
        # self.updateOdometry() deprecated for pose estimaaror with vision
        self.field.setRobotPose(self.updatePoseEstimator())
        if self.vision_enable:
            self.updateVisionData()
    
    def updatePeriodic(self) -> None:
        self.updateLogging()

        ##do sim stuff ig

    def drive(self,
              xSpeed:float,
              ySpeed:float,
              rot:float,
              fieldRelative:bool,
              ) -> None:
        """
        Parameters to construct desired ChassisSpeeds
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        #get module states through kinematics
        if fieldRelative:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.gyro.get_rotation_2d())
            )
        else:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                    ChassisSpeeds(xSpeed, ySpeed, rot)
            )
            
        #handle desired speeds > max speed
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds( swerveModuleStates, self.k_maxSpeed )

        #apply speeds
        for i, module in enumerate(self.modules):
            module.setDesiredState(swerveModuleStates[i])

    def drive_from_chassis_speeds(self, speeds:ChassisSpeeds):
        '''
        takes robot-relative ChassisSpeeds to drive
        '''
        #Convert ChassisSpeeds to SwerveModuleStates
        swerveModuleStates = self.kinematics.toSwerveModuleStates( speeds )

        #handle desired speeds > max speed
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds( swerveModuleStates, self.k_maxSpeed )

        #apply speeds
        for i, module in enumerate(self.modules):
            module.setDesiredState(swerveModuleStates[i])
    
    def updateOdometry(self) -> Pose2d:
        """update field relative position of robot"""
        return self.odometry.update(
            self.gyro.get_rotation_2d(),
            self.get_module_positions()
        )
    def updatePoseEstimator(self) -> Pose2d:
        return self.pose_estimator.update(
            self.gyro.get_rotation_2d(),
            self.get_module_positions()
        )
    def updateVisionData(self) -> None:
        data = self.vision.getVisionData()

        for pose, latency in data:
            self.pose_estimator.addVisionMeasurement(
                pose,
                getTime() - latency
            )

        if not self.first_vision_recieved and data != []:
            self.sync_gyro()
    
    def sync_gyro(self) -> None:
        self.gyro.set_yaw(self.pose_estimator.getEstimatedPosition().rotation().degrees())
    
    ## Getters
    def get_module_positions(self) -> tuple[SwerveModulePosition]:
        return tuple(module.getPosition() for module in self.modules)
    def get_module_states(self) -> tuple[SwerveModuleState]:
        return tuple(module.getState() for module in self.modules)
    
    def getPose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()
    
    ## Pathplanner Reqs

    #getPose

    def resetPose(self, pose:Pose2d) -> None:
        self.odometry.resetPose(pose)
        self.pose_estimator.resetPose(pose)
    
    def getChassisSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_module_states())
