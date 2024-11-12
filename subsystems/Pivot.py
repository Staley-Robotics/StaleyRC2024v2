import math

from commands2 import PIDSubsystem

from wpilib import SmartDashboard, RobotBase, Mechanism2d, Color8Bit, RobotController
from wpimath.controller import PIDController
from wpilib.simulation import SingleJointedArmSim
from wpimath.system import LinearSystem_2_1_2
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations, degrees, degreesToRotations, rotationsToDegrees
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, AbsoluteSensorRangeValue, SensorDirectionValue

class PivotConstants:
    MAX = 55.041
    AMP = 52.0
    SPEAKER = 50.0
    HANDOFF = 32.168
    TOSS = 25.0
    FLAT = 0.0
    MIN = -52.031

class Pivot(PIDSubsystem):
    # Constants
    kP:float = 3.0
    kI:float = 0.0
    kD:float = 0.0
    kS:float = 0.1
    kV:float = 0.0
    kTolerance:float = 0.01 # 0.0028 # Rotations 1.0 # In Degrees
    kGearRatio = 1 / 200

    # Motors
    m_motor:TalonFX = None
    m_encoder:CANcoder = None
    c_offset = -0.2138 # Rotations? ### -76.993 #degrees

    def __init__(self):
        # Motor
        m_motorCfg = TalonFXConfiguration()
        m_motorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        m_motorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        m_motorCfg.motor_output.duty_cycle_neutral_deadband = 0.001
        self.m_motor = TalonFX( 25, "canivore1" )
        self.m_motor.configurator.apply( m_motorCfg )
        self.voltOut = VoltageOut(0, use_timesync=True)
        self.dutyOut = DutyCycleOut(0, use_timesync=True)
        
        # Simulation Motor
        self.s_motor = DCMotor.falcon500FOC(1)
        self.s_motorSim = SingleJointedArmSim(
            LinearSystemId.singleJointedArmSystem( self.s_motor, 0.0005, 1.0 ),
            self.s_motor,
            1 / self.kGearRatio,
            0.2,
            -math.pi * 1 / self.kGearRatio,
            math.pi * 1 / self.kGearRatio,
            False,
            0
        )
        
        # Encoder
        m_encoderCfg = CANcoderConfiguration()
        m_encoderCfg.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        m_encoderCfg.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        if not RobotBase.isSimulation(): m_encoderCfg.magnet_sensor.magnet_offset = self.c_offset
        self.m_encoder = CANcoder( 26, "canivore1" )
        self.m_encoder.configurator.apply( m_encoderCfg )

        super().__init__(
            PIDController( self.kP, self.kI, self.kD ),
            self.m_encoder.get_position().value
        )

        # Controllers
        self._controller.setTolerance( self.kTolerance )
        self._controller.enableContinuousInput( -1.0, 1.0 )
        self.enable()

        # Mechanism Graphics / Logging
        self.mech = Mechanism2d( 28, 28, Color8Bit(0,0,0) )
        self.mechRoot = self.mech.getRoot( "PivotRoot", 5, 1 )
        self.mechAnchor = self.mechRoot.appendLigament( "PivotAnchor", 0, 0, 1, Color8Bit( 0, 0, 255 ) )
        self.mechPost = self.mechAnchor.appendLigament( "PivotPost", 10, 65, 2, Color8Bit( 0, 0, 255 ) )
        self.mechPostTop = self.mechPost.appendLigament( "PivotPostTop", 5, 0, 2, Color8Bit( 0, 0, 255 ) )
        self.mechFront = self.mechPost.appendLigament( "PivotFront", 4, 0, 2, Color8Bit( 0, 255, 0) )
        self.mechBack = self.mechPost.appendLigament( "PivotBack", 6, 0, 2, Color8Bit( 255, 0, 0) )
        SmartDashboard.putData( "PivotMech", self.mech )
        SmartDashboard.putData( "Pivot", self )
        SmartDashboard.putData( "PivotMotor", self.m_motor )
        SmartDashboard.putData( "PivotEncoder", self.m_encoder )
        SmartDashboard.putData( "PivotController", self._controller )

    def periodic(self) -> None:
        # Start of Subsystem Logging
        
        # Run
        super().periodic()

        # Post Subsystem Logging
        # Log Here
        offset = self.mechPost.getAngle()
        self.mechFront.setAngle( rotationsToDegrees( self.getMeasurement() ) - offset )
        self.mechBack.setAngle( rotationsToDegrees( self.getMeasurement() ) + 180 - offset )

    def simulationPeriodic(self) -> None:
        # Simulation Motor Deadband
        if self.atSetpoint() and abs( self.m_motor.get_duty_cycle().value ) <= 0.01:
            self.m_motor.set_control( self.voltOut.with_output( 0.0 ) )

        # Motor
        self.m_motor.sim_state.set_supply_voltage( RobotController.getBatteryVoltage() )
        self.s_motorSim.setInputVoltage( self.m_motor.sim_state.motor_voltage )
        self.s_motorSim.update(0.02)
        velocity = self.s_motorSim.getVelocity()
        #velocity = self.s_motor.speed( self.s_motor.torque( self.m_motor.sim_state.supply_current), self.m_motor.sim_state.motor_voltage )
        velocity = radiansToRotations( velocity )

        self.m_motor.sim_state.set_rotor_velocity( velocity )
        self.m_motor.sim_state.add_rotor_position( velocity * 0.02 )

        # CANcoder
        self.m_encoder.sim_state.set_velocity( velocity * self.kGearRatio )
        self.m_encoder.sim_state.add_position( velocity * self.kGearRatio * 0.02 )

    def setSetpoint(self, setpoint:degrees):
        # Limits the specific range (Protects mechanism)
        setpoint = min( max( setpoint, PivotConstants.MIN ), PivotConstants.MAX )
        setpoint = degreesToRotations( setpoint )
        return super().setSetpoint(setpoint)

    def useOutput(self, output:float, setpoint:float) -> None:
        self.m_motor.set_control( self.dutyOut.with_output( output ) )

    def getMeasurement(self) -> float:
        return self.m_encoder.get_position().value

    def atSetpoint(self) -> bool:
        return self._controller.atSetpoint()
