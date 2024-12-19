from rev import SparkMax, SparkMaxConfig, SparkMaxSim
from phoenix6.hardware import CANcoder # sim with CANcoder.sim_state
from phoenix6.configs import CANcoderConfiguration, MagnetSensorConfigs

from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians, rotationsPerMinuteToRadiansPerSecond

from wpilib import SmartDashboard

from math import pi

class SwerveModuleSim:

    kMaxVelocity = pi
    kMaxAcceleration = pi * 2

    k_drive_gear_ratio = 1 / 6.75
    k_wheel_radius = 0.0508

    def __init__(self,
                 ssName:str,
                 drive_motor_port:int,
                 turn_motor_port:int,
                 abs_encoder_port:int, abs_encoder_offset:float) -> None:
        self.ssName = ssName

        ##Data Vals
        self.turn_motor_inverted = True

        ##Physical Component Inits
        self.drive_motor = SparkMaxSim( drive_motor_port,  )
        self.turn_motor = SparkMaxSim( turn_motor_port, SparkMax.MotorType.kBrushless )

        self.abs_turn_encoder = CANcoder(abs_encoder_port, "canivore1")
        self.abs_encoder_offset = abs_encoder_offset

        self.drive_motor_encoder = self.drive_motor.getEncoder()
        self.turn_motor_encoder = self.turn_motor.getEncoder()

        ##Configs

        #Things to config probably:
        # change CAN timeout for cofigurating
        # current limiting & Voltage Compensation
        # set motor encoder positions, measurment periods, & depths

        drive_config = SparkMaxConfig().setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        drive_config.encoder.positionConversionFactor(2 * pi * self.k_drive_gear_ratio * self.k_wheel_radius)
        self.drive_motor.getEncoder()
        self.drive_motor.configure( drive_config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )
        turn_config = SparkMaxConfig().inverted(self.turn_motor_inverted)
        self.turn_motor.configure( turn_config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor.magnet_offset = -abs_encoder_offset
        self.abs_turn_encoder.configurator.apply(encoder_config)
        
        #PID Controllers
        # edit PID vals thru Sendable -> change here in code for persistance
        self.drivePID = PIDController(0.0, 0.0, 0.0)
        self.turnPID = PIDController(4.3,0.0,0.0)
        # self.turnPID = ProfiledPIDController(
        #     0.0, 0.0, 0.0,
        #     TrapezoidProfile.Constraints(
        #         self.kMaxVelocity,
        #         self.kMaxAcceleration
        #     )
        # )
        self.turnPID.enableContinuousInput(-pi,pi)

        self.driveFF = SimpleMotorFeedforwardMeters(0.0, 2.8235)
        # self.turnFF = SimpleMotorFeedforwardMeters(0, 0.0)

        SmartDashboard.putData(f"{ssName}drivePID", self.drivePID)
        SmartDashboard.putData(f"{ssName}turnPID", self.turnPID)

    ##Logging funcs
    def getDriveVelocity(self) -> float:
        # return self.drive_motor_encoder.getVelocity()
        velocity = self.drive_motor_encoder.getVelocity() * self.k_drive_gear_ratio
        velocity = rotationsPerMinuteToRadiansPerSecond(velocity)
        return velocity * self.k_wheel_radius
    
    ##Functional
    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive_motor_encoder.getVelocity(),
            Rotation2d(self.getAbsoluteEncoderPosition())
        )
    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.drive_motor_encoder.getPosition(),
            Rotation2d(self.getAbsoluteEncoderPosition())
        )

    def getAbsoluteEncoderPosition(self) -> float:
        return rotationsToRadians(self.abs_turn_encoder.get_absolute_position().value)# - self.abs_encoder_offset)

    def setDesiredState(self, desiredState:SwerveModuleState):
        encoderRotation = Rotation2d((self.getAbsoluteEncoderPosition()))

        #create optimized desired state
        state = desiredState
        state.optimize(encoderRotation)

        #make it drive slow when it has to turn a lot
        # SmartDashboard.putNumber("set velocity pre cos", state.speed)
        state.speed *= (state.angle - encoderRotation).cos()
        SmartDashboard.putNumber(f"SwerveModule-{self.ssName} set velocity", state.speed)
        SmartDashboard.putNumber(f"SwerveModule-{self.ssName} set angle", state.angle.radians())

        driveOutput = self.drivePID.calculate(
            self.drive_motor_encoder.getVelocity(), state.speed
        )
        driveFeedForward = self.driveFF.calculate(state.speed)

        #YAY PID
        turnOutput = self.turnPID.calculate(
            self.getAbsoluteEncoderPosition(), state.angle.radians()
        )
        # turnFeedForward = self.turnFF.calculate(self.turnPID.getSetpoint().velocity)

        self.drive_motor.setVoltage(driveOutput + driveFeedForward)
        self.turn_motor.setVoltage(turnOutput)# + turnFeedForward)

        