import typing
import math

from wpilib import RobotState, RobotBase
from wpimath.controller import PIDController, ProfiledPIDController, ProfiledPIDControllerRadians, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath import applyDeadband
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.system.plant import DCMotor
from wpimath.units import rotationsPerMinuteToRadiansPerSecond, rotationsToRadians, radiansToRotations, kSecondsPerMinute
from ntcore import NetworkTable, NetworkTableInstance

from rev import SparkMax, SparkRelativeEncoder, SparkMaxSim, SparkRelativeEncoderSim
from phoenix6.hardware import CANcoder
from phoenix6.sim import CANcoderSimState

class SwerveModuleConstants:
    class Drive:
        kWheelRadius:float = 0.0508
        kGearRatio:float = 1 / 6.75
        kP:float = 0 #0.1
        kI:float = 0
        kD:float = 0
        kS:float = 0 #0.1
        kV:float = 2.8235 #0.13
    
    class Turn:
        kGearRatio:float = 1 / (150/7)
        kP:float = 10.0 # 25.0
        kI:float = 0
        kD:float = 0
        kMaxAngularVelocity:float = math.pi
        kMaxAngularAcceleration:float = math.tau
        kS:float = 0
        kV:float = 0
    
    class NeoSim:
        kMaxRpm:float = radiansToRotations( DCMotor.NEO(1).freeSpeed ) * kSecondsPerMinute

class SwerveModule:
    # Variable Declaration
    __driveMotor:SparkMax = None
    __driveEncoder:SparkRelativeEncoder = None
    __drivePid:PIDController = None
    __driveFF:SimpleMotorFeedforwardMeters = None

    __turnMotor:SparkMax = None
    __turnEncoder:CANcoder = None
    __turnPid:ProfiledPIDControllerRadians = None
    __turnFF:SimpleMotorFeedforwardRadians = None

    __setpoint:SwerveModuleState = None

    def __init__(self, moduleId:int, driveId:int, turnId:int, encoderId:int, encoderOffset:float):
        # Module Name
        self.moduleId = moduleId
        
        # Drive Motor
        self.__driveMotor = SparkMax( driveId, SparkMax.MotorType.kBrushless )
        self.__driveEncoder = self.__driveMotor.getEncoder()
        self.__drivePid = PIDController( SwerveModuleConstants.Drive.kP, SwerveModuleConstants.Drive.kI, SwerveModuleConstants.Drive.kD )
        self.__drivePid.setTolerance( 0.01 )
        self.__driveFF = SimpleMotorFeedforwardMeters( SwerveModuleConstants.Drive.kS, SwerveModuleConstants.Drive.kV )

        # Turn Motor
        self.__turnMotor = SparkMax( turnId, SparkMax.MotorType.kBrushless )
        self.__turnMotorEncoder = self.__turnMotor.getEncoder()
        self.__turnEncoder = CANcoder( encoderId, "canivore1" )
        if not RobotBase.isSimulation():
            self.__turnEncoder.set_position( self.__turnEncoder.get_absolute_position().value_as_double - encoderOffset )
        self.__turnPid = PIDController( SwerveModuleConstants.Turn.kP, SwerveModuleConstants.Turn.kI, SwerveModuleConstants.Turn.kD )
        #self.__turnPid = ProfiledPIDControllerRadians( SwerveModuleConstants.Turn.kP, SwerveModuleConstants.Turn.kI, SwerveModuleConstants.Turn.kD, TrapezoidProfileRadians.Constraints( SwerveModuleConstants.Turn.kMaxAngularVelocity, SwerveModuleConstants.Turn.kMaxAngularAcceleration ) )
        self.__turnPid.enableContinuousInput( -math.pi, math.pi )
        self.__turnPid.setTolerance( 0.150 )
        self.__turnFF = SimpleMotorFeedforwardRadians( SwerveModuleConstants.Turn.kS, SwerveModuleConstants.Turn.kV )

        # Drive Motor Simulation Setup
        self.__driveMotorSim = SparkMaxSim( self.__driveMotor, DCMotor.NEO(1) )
        self.__driveMotorSim.getAbsoluteEncoderSim().setPositionConversionFactor(1)
        self.__driveMotorSim.getAbsoluteEncoderSim().setVelocityConversionFactor(1)
        self.__driveMotorSim.getRelativeEncoderSim().setPositionConversionFactor(1)
        self.__driveMotorSim.getRelativeEncoderSim().setVelocityConversionFactor(1)
        
        # Turn Motor Simulation Setup
        self.__turnMotorSim = SparkMaxSim( self.__turnMotor, DCMotor.NEO(1) )
        self.__turnMotorSim.getAbsoluteEncoderSim().setPositionConversionFactor(1)
        self.__turnMotorSim.getAbsoluteEncoderSim().setVelocityConversionFactor(1)
        self.__turnMotorSim.getRelativeEncoderSim().setPositionConversionFactor(1)
        self.__turnMotorSim.getRelativeEncoderSim().setVelocityConversionFactor(1)
        
        # CANcoder Simulation Setup
        self.__turnEncoderSim = self.__turnEncoder.sim_state

        # Default Desired State
        self.__setpoint = SwerveModuleState(0, Rotation2d(0))

        # Logging
        self.__logger = NetworkTableInstance.getDefault().getTable( f"/Logging/SwerveDrive/SwerveModule/{self.moduleId}" )

    def run(self) -> None:
        # Drive Motor
        driveVelocity = self.__getDriveVelocity( self.__driveEncoder.getVelocity() )
        driveOutput = self.__drivePid.calculate( driveVelocity, self.__setpoint.speed )
        driveOutputFF = self.__driveFF.calculate( self.__setpoint.speed )
        driveOut = driveOutput + driveOutputFF
        if RobotBase.isSimulation() and abs( driveOut ) < 0.12: driveOut = 0.0
        self.__driveMotor.setVoltage( driveOut )

        # Turn Motor
        turnOutput = self.__turnPid.calculate( self.__getTurnEncoderRadians(), self.__setpoint.angle.radians() )
        turnOutputFF = self.__turnFF.calculate( self.__setpoint.angle.radians() )
        turnOut = turnOutput + turnOutputFF
        if RobotBase.isSimulation() and abs( turnOut ) < 0.12: turnOut = 0.0
        self.__turnMotor.setVoltage( turnOut )

        # Logging
        self.__logger.putNumber( "DriveInput", self.__driveMotor.get() )
        self.__logger.putNumber( "DriveOutput", self.__driveMotor.getAppliedOutput() )
        self.__logger.putNumber( "DrivePosition_r", self.__driveEncoder.getPosition() )
        self.__logger.putNumber( "DriveVelocity_rpm", self.__driveEncoder.getVelocity() )

        self.__logger.putNumber( "TurnInput", self.__turnMotor.get() )
        self.__logger.putNumber( "TurnOutput", self.__turnMotor.getAppliedOutput() )
        self.__logger.putNumber( "TurnPosition_r", self.__turnMotorEncoder.getPosition() )
        self.__logger.putNumber( "TurnVelocity_rpm", self.__turnMotorEncoder.getVelocity() )

        self.__logger.putNumber( "EncoderPosition_r", self.__turnEncoder.get_position().value )
        self.__logger.putNumber( "EncoderVelocity_rps", self.__turnEncoder.get_velocity().value )

    def runSim(self) -> None:
        # Drive Motor Position and Velocity
        driveRpm = SwerveModuleConstants.NeoSim.kMaxRpm * self.__driveMotor.get()
        self.__driveMotorSim.getRelativeEncoderSim().iterate( driveRpm, 0.02 )
        self.__driveMotorSim.getRelativeEncoderSim().setVelocity( driveRpm )
        self.__driveMotorSim.getAbsoluteEncoderSim().setVelocity( driveRpm )
        self.__driveMotorSim.getAbsoluteEncoderSim().setPosition( self.__driveMotorSim.getRelativeEncoderSim().getPosition() % 1 )
        
        # Turn Motor Position and Velocity
        turnRpm = SwerveModuleConstants.NeoSim.kMaxRpm * self.__turnMotor.get()
        self.__turnMotorSim.getRelativeEncoderSim().iterate( turnRpm, 0.02 )
        self.__turnMotorSim.getRelativeEncoderSim().setVelocity( turnRpm )
        self.__turnMotorSim.getAbsoluteEncoderSim().setVelocity( turnRpm )
        self.__turnMotorSim.getAbsoluteEncoderSim().setPosition( self.__turnMotorSim.getRelativeEncoderSim().getPosition() % 1 )

        # CANcoder Velocity and Position
        canRps = turnRpm * SwerveModuleConstants.Turn.kGearRatio / kSecondsPerMinute
        self.__turnEncoderSim.set_velocity( canRps )
        self.__turnEncoderSim.add_position( canRps * 0.02 )

    def setState(self, desiredState:SwerveModuleState) -> None:
        # Optimize
        currentRotation = self.__getTurnEncoderRotation()
        self.__setpoint = desiredState
        self.__setpoint.optimize( currentRotation )
        self.__setpoint.cosineScale( currentRotation ) # Smooth Out Turning

    def getState(self) -> SwerveModuleState:
        driveVelocity = self.__getDriveVelocity( self.__driveEncoder.getVelocity() )
        return SwerveModuleState(
            driveVelocity,
            self.__getTurnEncoderRotation()
        )

    def getPosition(self) -> SwerveModulePosition:
        driveDistance = self.__getDriveDistance( self.__driveEncoder.getPosition() )
        return SwerveModulePosition(
            driveDistance,
            self.__getTurnEncoderRotation()
        )

    def __getDriveDistance(self, rotations:float) -> float:
        wheelRotations = rotations * SwerveModuleConstants.Drive.kGearRatio
        wheelRadians = rotationsToRadians( wheelRotations )
        meters = wheelRadians * SwerveModuleConstants.Drive.kWheelRadius
        return meters

    def __getDriveVelocity(self, rotationsPerMinute:float) -> float:
        wheelRotationsPerMin = rotationsPerMinute * SwerveModuleConstants.Drive.kGearRatio
        wheelRadiansPerSec = rotationsPerMinuteToRadiansPerSecond( wheelRotationsPerMin )
        metersPerSec = wheelRadiansPerSec * SwerveModuleConstants.Drive.kWheelRadius
        return metersPerSec

    def __getTurnEncoderRotation(self) -> Rotation2d:
        return Rotation2d.fromRotations( self.__turnEncoder.get_position().value_as_double )

    def __getTurnEncoderRadians(self) -> float:
        return rotationsToRadians( self.__turnEncoder.get_position().value_as_double )
