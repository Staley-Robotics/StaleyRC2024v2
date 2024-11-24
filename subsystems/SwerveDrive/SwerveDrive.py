from wpilib import RobotState, getTime
from wpimath.geometry import Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator

from commands2 import Subsystem

from ntcore import NetworkTable, NetworkTableInstance
# from ntcore.util import ntproperty

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
        self.poseTopic = self.m_logging.getStructTopic("robot pose odometry", Pose2d).publish()

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


    def periodic(self) -> None:
        ## Logging
        self.m_logging.putNumber( "Gyro value", self.gyro.get_rotation_2d().degrees() )
        self.poseTopic.set(self.pose_estimator.getEstimatedPosition())
        self.m_logging.putNumber( f"SwerveModule0 velocity measured", self.modules[0].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule0 angle measured", self.modules[0].getAbsoluteEncoderPosition() )
        self.m_logging.putNumber( f"SwerveModule1 velocity measured", self.modules[1].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule1 angle measured", self.modules[1].getAbsoluteEncoderPosition() )
        self.m_logging.putNumber( f"SwerveModule2 velocity measured", self.modules[2].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule2 angle measured", self.modules[2].getAbsoluteEncoderPosition() )
        self.m_logging.putNumber( f"SwerveModule3 velocity measured", self.modules[3].getDriveVelocity() )
        self.m_logging.putNumber( f"SwerveModule3 angle measured", self.modules[3].getAbsoluteEncoderPosition() )
        
        
        self.updateOdometry()
        self.updatePoseEstimator()
        self.updateVisionData()

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
                ChassisSpeeds.discretize(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.gyro.get_rotation_2d()),
                    0.02
                )
            )
        else:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    ChassisSpeeds(xSpeed, ySpeed, rot),
                    0.02
                )
            )
            
        #handle desired speeds > max speed
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds( swerveModuleStates, self.k_maxSpeed )

        for i, module in enumerate(self.modules):
            module.setDesiredState(swerveModuleStates[i])
        # self.modules[2].setDesiredState(swerveModuleStates[2])
        
    
    def updateOdometry(self) -> None:
        """update field relative position of robot"""
        self.odometry.update(
            self.gyro.get_rotation_2d(),
            self.get_module_positions()
        )
    def updatePoseEstimator(self) -> None:
        self.pose_estimator.update(
            self.gyro.get_rotation_2d(),
            self.get_module_positions()
        )
    def updateVisionData(self) -> None:
        data = self.vision.getLastUpdates()
        for pose, latency in data:
            self.pose_estimator.addVisionMeasurement(
                pose,
                getTime() - latency
            )
    
    ## Getters
    def get_module_positions(self) -> tuple[SwerveModulePosition]:
        return tuple(module.getPosition() for module in self.modules)
    
    def get_pose2d(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()