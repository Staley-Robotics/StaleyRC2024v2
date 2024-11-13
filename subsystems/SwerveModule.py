import typing
import math

from wpilib import RobotState, RobotBase
from wpimath.controller import PIDController, ProfiledPIDController, ProfiledPIDControllerRadians, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath import applyDeadband
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.system.plant import DCMotor
from wpimath.units import rotationsPerMinuteToRadiansPerSecond, rotationsToRadians, radiansToRotations
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

class SwerveModule:
    # Variable Declaration
    m_driveMotor:SparkMax = None
    m_driveEncoder:SparkRelativeEncoder = None
    c_drivePid:PIDController = None
    c_driveFF:SimpleMotorFeedforwardMeters = None

    m_turnMotor:SparkMax = None
    m_turnEncoder:CANcoder = None
    c_turnPid:ProfiledPIDControllerRadians = None
    c_turnFF:SimpleMotorFeedforwardRadians = None

    o_desiredState:SwerveModuleState = None

    def __init__(self, driveId:int, turnId:int, encoderId:int, encoderOffset:float):
        # Drive Motor
        self.m_driveMotor = SparkMax( driveId, SparkMax.MotorType.kBrushless )
        self.m_driveEncoder = self.m_driveMotor.getEncoder()
        self.c_drivePid = PIDController( SwerveModuleConstants.Drive.kP, SwerveModuleConstants.Drive.kI, SwerveModuleConstants.Drive.kD )
        self.c_drivePid.setTolerance( 0.01 )
        self.c_driveFF = SimpleMotorFeedforwardMeters( SwerveModuleConstants.Drive.kS, SwerveModuleConstants.Drive.kV )

        # Turn Motor
        self.m_turnMotor = SparkMax( turnId, SparkMax.MotorType.kBrushless )
        self.m_turnEncoder = CANcoder( encoderId, "canivore1" )
        if not RobotBase.isSimulation():
            self.m_turnEncoder.set_position( self.m_turnEncoder.get_absolute_position().value_as_double - encoderOffset )
        self.c_turnPid = PIDController( SwerveModuleConstants.Turn.kP, SwerveModuleConstants.Turn.kI, SwerveModuleConstants.Turn.kD )
        #self.c_turnPid = ProfiledPIDControllerRadians( SwerveModuleConstants.Turn.kP, SwerveModuleConstants.Turn.kI, SwerveModuleConstants.Turn.kD, TrapezoidProfileRadians.Constraints( SwerveModuleConstants.Turn.kMaxAngularVelocity, SwerveModuleConstants.Turn.kMaxAngularAcceleration ) )
        self.c_turnPid.enableContinuousInput( -math.pi, math.pi )
        self.c_turnPid.setTolerance( 0.150 )
        self.c_turnFF = SimpleMotorFeedforwardRadians( SwerveModuleConstants.Turn.kS, SwerveModuleConstants.Turn.kV )

        # Simulation Objects
        self.m_driveMotorSim = SparkMaxSim( self.m_driveMotor, DCMotor.NEO(1) )
        self.m_driveEncoderSim = self.m_driveMotorSim.getRelativeEncoderSim()
        self.m_turnMotorSim = SparkMaxSim( self.m_turnMotor, DCMotor.NEO(1) )
        self.m_turnMotorEncoderSim = self.m_turnMotorSim.getRelativeEncoderSim()
        self.m_turnEncoderSim = self.m_turnEncoder.sim_state

        # Default Desired State
        self.o_desiredState = SwerveModuleState(0, Rotation2d(0))

    def run(self) -> None:
        # Drive Motor
        driveVelocity = self.__getDriveVelocity( self.m_driveEncoder.getVelocity() )
        driveOutput = self.c_drivePid.calculate( driveVelocity, self.o_desiredState.speed )
        driveOutputFF = self.c_driveFF.calculate( self.o_desiredState.speed )
        driveOut = driveOutput + driveOutputFF
        if abs( driveOut ) < 0.12: driveOut = 0.0
        self.m_driveMotor.setVoltage( driveOut )

        # Turn Motor
        turnOutput = self.c_turnPid.calculate( self.__getTurnEncoderRadians(), self.o_desiredState.angle.radians() )
        turnOutputFF = self.c_turnFF.calculate( self.o_desiredState.angle.radians() )
        turnOut = turnOutput + turnOutputFF
        if abs( turnOut ) < 0.12: turnOut = 0.0
        self.m_turnMotor.setVoltage( turnOut )

    def runSim(self) -> None:
        # Drive Motor Velocity
        #driveRotationsPerMinute = self.o_desiredState.speed / self.__getDriveVelocity( 1 ) # Convert this to voltes???
        driveRpm = 5676.0 * self.m_driveMotor.get()
        driveRpm = min( max( driveRpm, -5676.0 ), 5676.0 )
        self.m_driveEncoderSim.setVelocity( driveRpm )

        # Drive Motor Position
        driveRp20ms = driveRpm / 60 * 0.02
        self.m_driveEncoderSim.setPosition( self.m_driveEncoderSim.getPosition() + driveRp20ms )

        # Turn Motor Velocity
        turnRpm = 5676.0 * self.m_turnMotor.get()
        turnRpm = min( max( turnRpm, -5676.0 ), 5676.0 )
        self.m_turnMotorEncoderSim.setVelocity( turnRpm )

        # Turn Motor Position
        turnRp20ms = turnRpm / 60 * 0.02
        self.m_turnMotorEncoderSim.setPosition( self.m_turnMotorEncoderSim.getPosition() + turnRp20ms )

        # CANcoder Velocity
        canRpm = turnRpm * SwerveModuleConstants.Turn.kGearRatio 
        canRps = canRpm / 60
        self.m_turnEncoderSim.set_velocity( canRps )
        
        # CANcoder Position
        canRp20ms = canRps * 0.02
        self.m_turnEncoderSim.add_position( canRp20ms )

    def setState(self, desiredState:SwerveModuleState) -> None:
        # Optimize
        currentRotation = self.__getTurnEncoderRotation()
        self.o_desiredState = desiredState
        self.o_desiredState.optimize( currentRotation )
        self.o_desiredState.cosineScale( currentRotation ) # Smooth Out Turning

    def getState(self) -> SwerveModuleState:
        driveVelocity = self.__getDriveVelocity( self.m_driveEncoder.getVelocity() )
        return SwerveModuleState(
            driveVelocity,
            self.__getTurnEncoderRotation()
        )

    def getPosition(self) -> SwerveModulePosition:
        driveDistance = self.__getDriveDistance( self.m_driveEncoder.getPosition() )
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
        return Rotation2d.fromRotations( self.m_turnEncoder.get_position().value_as_double )

    def __getTurnEncoderRadians(self) -> float:
        return rotationsToRadians( self.m_turnEncoder.get_position().value_as_double )
