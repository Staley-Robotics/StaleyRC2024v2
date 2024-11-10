from commands2 import PIDSubsystem

from wpilib import SmartDashboard
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, MagnetSensorConfigs
from phoenix6.signals.spn_enums import *

class Pivot(PIDSubsystem):
    # Constants
    kP:float = 0.0
    kI:float = 0.0
    kD:float = 0.0
    kS:float = 0.0
    kV:float = 0.0
    kTolerance:float = 0.25 # In Degrees

    # Motors
    m_motor:TalonFX = None
    m_encoder:CANcoder = None
    c_offset = -0.2138 # Rotations? ### -76.993 #degrees

    def __init__(self):
        # Motor
        m_motorCfg = TalonFXConfiguration()
        m_motorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        m_motorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        m_motorCfg.motor_output.duty_cycle_neutral_deadband = 0.005
        self.m_motor = TalonFX( 25, "canivore1" )
        self.m_motor.configurator.apply( m_motorCfg )

        # Encoder
        m_encoderCfg = CANcoderConfiguration()
        m_encoderCfg.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        m_encoderCfg.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        m_encoderCfg.magnet_sensor.magnet_offset = self.c_offset
        self.m_encoder = CANcoder( 26, "canivore1" )
        self.m_encoder.configurator.apply( m_encoderCfg )

        super().__init__(
            PIDController( self.kP, self.kI, self.kD ),
            self.m_encoder.get_position().value
        )

        # Controllers
        self._controller.setTolerance( self.kTolerance )
        super().setSetpoint( self.m_encoder.get_position().value * 360.0 )
        self.m_feedforward = SimpleMotorFeedforwardMeters( self.kS, self.kV )

        SmartDashboard.putData( "Pivot", self )

    def periodic(self) -> None:
        # Start of Subsystem Logging
        
        # Run
        super().periodic()

        # Post Subsystem Logging
        # Log Here

    def useOutput(self, output:float, setpoint:float) -> None:
        self.m_motor.set( output + self.m_feedforward.calculate(setpoint) )

    def getMeasurement(self) -> float:
        return self.m_encoder.get_position().value * 360.0

    def atSetpoint(self) -> float:
        return self._controller.atSetpoint()