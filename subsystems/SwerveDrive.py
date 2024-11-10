import typing
import math

from commands2 import Subsystem
from wpilib import RobotState, SmartDashboard, Field2d
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState, SwerveDrive4Odometry, ChassisSpeeds
from ntcore import NetworkTable, NetworkTableInstance

from phoenix6.hardware import Pigeon2

from subsystems.SwerveModule import SwerveModule

class SwerveDrive(Subsystem):
    # Static Constants
    kMaxSpeed = 4.4 # Meters Per Second
    kRotationSpeed = math.pi # Rotations Per Second

    # Variable Declaration
    m_modules:typing.Tuple[ SwerveModule, SwerveModule, SwerveModule, SwerveModule ] = None
    m_gyro:Pigeon2 = None
    m_kinematics:SwerveDrive4Kinematics = None
    m_odometry:SwerveDrive4Odometry = None
    m_logging:NetworkTable = None

    # Settings
    s_FieldRelative = True
    s_MaxSpeedPercent = 1.0
    s_MaxRotationPercent = 1.0

    # Initialization
    def __init__(self) -> None:
        self.setName( "SwerveDrive" )

        self.m_modules = [
            SwerveModule( 7, 8, 18, 97.471 ),
            SwerveModule( 1, 2, 12, 5.361 ),
            SwerveModule( 5, 6, 16, 298.828 ),
            SwerveModule( 3, 4, 14, 60.557 )
        ]

        self.m_gyro = Pigeon2( 9, "canivore1" )

        self.m_kinematics = SwerveDrive4Kinematics(
            Translation2d( 0.2667, 0.2667 ),
            Translation2d( 0.2667, -0.2667 ),
            Translation2d( -0.2667, 0.2667 ),
            Translation2d( -0.2667, -0.2667 )
        )

        self.m_odometry = SwerveDrive4Odometry(
            self.m_kinematics,
            self.m_gyro.getRotation2d(),
            self.__getModulePositions(),
            Pose2d( Translation2d(0,0), Rotation2d(0) )
        )

        self.stop()

        self.m_field = Field2d()
        SmartDashboard.putData("Field", self.m_field)

        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/SwerveDrive")

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.m_logging.putNumber( "SubsystemData", 0.0 )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        
        # Run SwerveModules
        self.m_modules[0].run()
        self.m_modules[1].run()
        self.m_modules[2].run()
        self.m_modules[3].run()
        
        # Update Odometry
        self.l_pose =  self.m_odometry.update(
            self.m_gyro.getRotation2d(),
            self.__getModulePositions()
        )
        
        self.m_field.setRobotPose( self.l_pose )
        # Logging: Write Post Operation Information
        #self.m_logging.putNumber( "Setpoint", self.getSetpoint() )
        #self.m_logging.putNumber( "Measured", self.m_system )

    # Simulation Periodic Loop
    def simulationPeriodic(self) -> None:
        # Run SwerveModules
        self.m_modules[0].runSim()
        self.m_modules[1].runSim()
        self.m_modules[2].runSim()
        self.m_modules[3].runSim()

        # Update Gyro
        moduleStates = self.__getModuleStates()
        actualSpeed = self.m_kinematics.toChassisSpeeds( moduleStates )
        yawPer20ms = actualSpeed.omega_dps * 0.02
        self.m_gyro.sim_state.add_yaw( yawPer20ms )

    # Stop the Subsystem
    def stop(self) -> None:
        self.runChassisSpeeds( ChassisSpeeds( 0.0, 0.0, 0.0 ) )
 
    # Run By Percentage
    def runPercentInputs(self, x:float, y:float, omega:float) -> None:
        # Range Tolerances
        x = min( max( x, -1.0 ), 1.0 )
        y = min( max( y, -1.0 ), 1.0 )
        omega = min( max( omega, -1.0 ), 1.0 )

        xSpeed = x * self.s_MaxSpeedPercent * self.kMaxSpeed
        ySpeed = y * self.s_MaxSpeedPercent * self.kMaxSpeed
        omegaSpeed = omega * self.s_MaxRotationPercent * self.kRotationSpeed

        cSpeed = (
            ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, omegaSpeed, self.m_gyro.getRotation2d() )
            if self.s_FieldRelative
            else ChassisSpeeds( xSpeed, ySpeed, omegaSpeed )
        )
        self.runChassisSpeeds( cSpeed )

    # Run By Chassis Speeds
    def runChassisSpeeds(self, cSpeeds:ChassisSpeeds) -> None:
        newCspeeds = ChassisSpeeds.discretize( cSpeeds, 0.02 )
        moduleStates = self.m_kinematics.toSwerveModuleStates( newCspeeds )
        self.runModuleStates( moduleStates )

    # Run By SwerveModuleStates
    def runModuleStates(self, swerveStates:typing.Tuple[ SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState ]) -> None:
        newStates = SwerveDrive4Kinematics.desaturateWheelSpeeds( swerveStates, self.kMaxSpeed )
        self.m_modules[0].setState(newStates[0])
        self.m_modules[1].setState(newStates[1])
        self.m_modules[2].setState(newStates[2])
        self.m_modules[3].setState(newStates[3])

    def __getModulePositions(self) -> typing.Tuple[ SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        return [
            self.m_modules[0].getPosition(),
            self.m_modules[1].getPosition(),
            self.m_modules[2].getPosition(),
            self.m_modules[3].getPosition()
        ]
    
    def __getModuleStates(self) -> typing.Tuple[ SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return [
            self.m_modules[0].getState(),
            self.m_modules[1].getState(),
            self.m_modules[2].getState(),
            self.m_modules[3].getState()
        ]
    
    def getMaxSpeed(self) -> float:
        return self.kMaxSpeed
    
    def getMaxRotation(self) -> float:
        return self.kRotationSpeed

    def getLimitedMaxSpeed(self) -> float:
        return self.kMaxSpeed * self.s_MaxSpeedPercent
    
    def getLimitedMaxRotation(self) -> float:
        return self.kRotationSpeed * self.s_MaxRotationPercent

    def getFieldRelative(self) -> bool:
        return self.s_FieldRelative
    
    def setFieldRelative(self, isFieldRelative:bool) -> None:
        self.s_FieldRelative = isFieldRelative

    def toggleFieldRelative(self) -> None:
        self.s_FieldRelative = not self.s_FieldRelative