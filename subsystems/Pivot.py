import math

from commands2 import PIDSubsystem

from ntcore import NetworkTableInstance, NetworkTable
from wpilib import SmartDashboard, RobotBase, Mechanism2d, Color8Bit, RobotController
from wpimath.controller import PIDController, SimpleMotorFeedforwardRadians
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees, degreesToRotations, rotationsToDegrees

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, AbsoluteSensorRangeValue, SensorDirectionValue

class PivotPositions:
    MAX = 55.041
    AMP = 52.0
    SPEAKER = 50.0
    HANDOFF = 32.168
    TOSS = 25.0
    FLAT = 0.0
    MIN = -52.031

class PivotConstants:
    # Constants
    kP:float = 25.0
    kI:float = 0.0
    kD:float = 0.0
    kS:float = 0.1
    kV:float = 0.0
    kTolerance:float = 0.01 # 0.0028 # Rotations 1.0 # In Degrees
    
    kGearRatio:float = 1 / 200
    kOffsetRotations:float = -0.2138 # Rotations? ### -76.993 #degrees

    class FalconSim:
        kMaxRps = radiansToRotations( DCMotor.falcon500FOC(1).freeSpeed )

class Pivot(PIDSubsystem):
    # Motors
    __motor:TalonFX = None
    __encoder:CANcoder = None
    __logger:NetworkTable = None
    c_offset = -0.2138 

    def __init__(self):
        # Motor
        motorCfg = TalonFXConfiguration()
        motorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        motorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        motorCfg.motor_output.duty_cycle_neutral_deadband = 0.001
        self.__motor = TalonFX( 25, "canivore1" )
        self.__motor.configurator.apply( motorCfg )
        self.voltOut = VoltageOut(0, use_timesync=True)
        self.dutyOut = DutyCycleOut(0, use_timesync=True)
                
        # Encoder
        encoderCfg = CANcoderConfiguration()
        encoderCfg.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        encoderCfg.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        if not RobotBase.isSimulation(): encoderCfg.magnet_sensor.magnet_offset = PivotConstants.kOffsetRotations
        self.__encoder = CANcoder( 26, "canivore1" )
        self.__encoder.configurator.apply( encoderCfg )

        # PID Controller
        pidController = PIDController( PivotConstants.kP, PivotConstants.kI, PivotConstants.kD )
        pidController.setTolerance( PivotConstants.kTolerance )
        pidController.enableContinuousInput( -0.5, 0.5 )

        super().__init__(
            pidController,
            self.__encoder.get_position().value
        )

        # Enable Subsystem PIDController
        self.enable()

        # Mechanism Graphics / Logging
        self.mech = Mechanism2d( 28, 28, Color8Bit(0,0,0) )
        self.mechRoot = self.mech.getRoot( "PivotRoot", 5, 1 )
        self.mechAnchor = self.mechRoot.appendLigament( "PivotAnchor", 0, 0, 1, Color8Bit( 0, 0, 255 ) )
        self.mechPost = self.mechAnchor.appendLigament( "PivotPost", 10, 65, 2, Color8Bit( 0, 0, 255 ) )
        self.mechPostTop = self.mechPost.appendLigament( "PivotPostTop", 5, 0, 2, Color8Bit( 0, 0, 255 ) )
        self.mechFront = self.mechPost.appendLigament( "PivotFront", 4, 0, 2, Color8Bit( 0, 255, 0) )
        self.mechBack = self.mechPost.appendLigament( "PivotBack", 6, 0, 2, Color8Bit( 255, 0, 0) )
        SmartDashboard.putData( "Pivot", self )
        SmartDashboard.putData( "PivotMech", self.mech )
        SmartDashboard.putData( "PivotPid", self._controller )

        self.__logger:NetworkTable = NetworkTableInstance.getDefault().getTable( "/Logging/Pivot" )
        self.__measured:NetworkTable = NetworkTableInstance.getDefault().getTable( "/RealOutputs/Pivot" )

    def periodic(self) -> None:
        # Input Logging
        self.__logger.putNumber( "MotorInput", self.__motor.get() )
        self.__logger.putNumber( "MotorOutput", self.__motor.get_motor_voltage().value )
        self.__logger.putNumber( "MotorPosition_r", self.__motor.get_position().value )
        self.__logger.putNumber( "MotorVelocity_rps", self.__motor.get_velocity().value )
        self.__logger.putNumber( "EncoderPosition_r", self.__encoder.get_position().value )
        self.__logger.putNumber( "EncoderVelocity_rps", self.__encoder.get_velocity().value )
        
        # Run
        super().periodic()

        # Visualization
        offset = self.mechPost.getAngle()
        self.mechFront.setAngle( rotationsToDegrees( self.getMeasurement() ) - offset )
        self.mechBack.setAngle( rotationsToDegrees( self.getMeasurement() ) + 180 - offset )

        # Output Logging
        self.__measured.putNumber( "TargetAngle", rotationsToDegrees( self.getSetpoint() ) )
        self.__measured.putNumber( "ActualAngle", rotationsToDegrees( self.getMeasurement() ) )

    def simulationPeriodic(self) -> None:
        # Simulation Motor Deadband
        if self.atSetpoint() and abs( self.__motor.get_duty_cycle().value ) <= 0.011:
            self.__motor.set_control( self.dutyOut.with_output( 0.0 ) )

        # Motor
        self.__motor.sim_state.set_supply_voltage( RobotController.getBatteryVoltage() )
        velocity = self.__motor.sim_state.motor_voltage / 12 * PivotConstants.FalconSim.kMaxRps

        self.__motor.sim_state.set_rotor_velocity( velocity )
        self.__motor.sim_state.add_rotor_position( velocity * 0.02 )

        # CANcoder
        self.__encoder.sim_state.set_velocity( velocity * PivotConstants.kGearRatio )
        self.__encoder.sim_state.add_position( velocity * PivotConstants.kGearRatio * 0.02 )

    def setSetpoint(self, setpoint:degrees):
        # Limits the specific range (Protects mechanism)
        setpoint = min( max( setpoint, PivotPositions.MIN ), PivotPositions.MAX )
        setpoint = degreesToRotations( setpoint )
        return super().setSetpoint(setpoint)

    def useOutput(self, output:float, setpoint:float) -> None:
        # Placeholder: Feed Forward Not Implemented
        feedforward = SimpleMotorFeedforwardRadians( 0, 0, 0 ).calculate( setpoint ) 

        # Sets the motor speed
        self.__motor.set_control( self.dutyOut.with_output( output ) )

    def getMeasurement(self) -> float:
        return self.__encoder.get_position().value

    def atSetpoint(self) -> bool:
        return self._controller.atSetpoint()
