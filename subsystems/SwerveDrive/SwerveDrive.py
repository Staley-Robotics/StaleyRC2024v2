from wpilib import RobotState
import wpimath.geometry
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds

from commands2 import Subsystem

# from phoenix6.hardware import Pigeon2
from .W_Pigeon2 import W_Pigeon2

from ntcore import NetworkTable, NetworkTableInstance

from .SwerveModule import SwerveModule
from util.Tunable import Tunable

class SwerveDrive(Subsystem):
    # Variable Declaration
    m_system:int = None
    m_logging:NetworkTable = None

    k_maxSpeed:float = 3.0 # meters / sec

    def __init__(self, sysId:int, modules:list[SwerveModule], gyro:W_Pigeon2) -> None:
        ##Subsystem Inits
        self.setName("SwerveDrive")

        ##Logging Inits
        self.m_system = sysId
        self.m_value = 0.0
        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/SwerveDrive")

        # self.test_tunable = Tunable("/Config/SwerveDrive", "test tunable", 0.0, lambda: self.m_logging.putNumber("does tunable work?", self.test_tunable.get()))

        # self.m_logging.putNumber("/Config/SwervDrive/Override drive FF", 0.0)

        ##Swerve Inits
        #SwerveModule offsets from center, in meters
        fl_offset = wpimath.geometry.Translation2d(0.2667,  0.2667)
        fr_offset = wpimath.geometry.Translation2d(0.2667, -0.2667)
        bl_offset = wpimath.geometry.Translation2d(-0.2667,  0.2667)
        br_offset = wpimath.geometry.Translation2d(-0.2667, -0.2667)

        #low-level components
        self.modules = modules
        self.gyro = gyro

        #converts between chassis velocity and Swerve Module States
        self.kinematics = SwerveDrive4Kinematics(
            fl_offset,
            fr_offset,
            bl_offset,
            br_offset
        )

        #tracks field position, args are mostly starting position
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.get_rotation_2d(),
            [
                module.getPosition() for module in self.modules
            ]
        )

        # self.gyro.configurator.refresh()

    def periodic(self) -> None:
        ## Logging: Write Current Subsystem State
        self.m_logging.putNumber( "Gyro value", self.gyro.get_rotation_2d().degrees() )
        for module in self.modules:
            self.m_logging.putNumber( f"SwerveModule{module.ssName} velocity measured", module.getDriveVelocity() )
            self.m_logging.putNumber( f"SwerveModule{module.ssName} angle measured", module.getAbsoluteEncoderPosition() )

        # self.m_logging.putNumber( "does this tunable work?", self.)
        # Update Tunables        
        

        # Run Subsystem: Set New State To Subsystem
        # if RobotState.isDisabled():
        #     self.stop()
        # else:
        #     self.run()
        
        # # Logging: Write Post Operation Information
        # self.m_logging.putNumber( "Setpoint", self.getSetpoint() )
        # self.m_logging.putNumber( "Measured", self.m_system )

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
            [
                module.getPosition() for module in self.modules
            ]
        )

    # # Run the Subsystem
    # def run(self) -> None:
    #     pass

    # # Stop the Subsystem
    # def stop(self) -> None:
    #     pass

    # # Set the Desired State Value
    # def setSetpoint(self, value:float) -> None:
    #     self.m_value = value

    # # Get the Desired State Value
    # def getSetpoint(self) -> float:
    #     return self.m_value
    
    # # Check if Subsystem is at the Desired State
    # def atSetpoint(self) -> bool:
    #     return False