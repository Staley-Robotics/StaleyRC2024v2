import statistics

from commands2 import Subsystem

from wpilib import SmartDashboard, Mechanism2d, Color8Bit, RobotController, DigitalInput
from wpimath.units import rotationsToDegrees
from ntcore import NetworkTable, NetworkTableInstance

from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue

class IntakeConstants:
    EJECT:float = -1.0
    STOP:float = 0.0
    HANDOFF:float = 0.5
    PICKUP:float = 0.5

class Intake(Subsystem):
    # Variable Declaration
    __setpoint:float = 0.0
    m_topMotor:TalonFX = None
    m_bottomMotor:TalonFX = None
    m_irBeam:DigitalInput = None
    m_logging:NetworkTable = None

    # Initialization
    def __init__(self) -> None:
        # Top Motor
        self.m_topMotorCfg = TalonFXConfiguration()
        self.m_topMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.m_topMotorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        self.m_topMotor = TalonFX(20, "canivore1")
        self.m_topMotor.configurator.apply( self.m_topMotorCfg )

        # Bottom Motor
        self.m_bottomMotorCfg = TalonFXConfiguration()
        self.m_bottomMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.m_bottomMotorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        self.m_bottomMotor = TalonFX(21, "canivore1")
        self.m_bottomMotor.configurator.apply( self.m_bottomMotorCfg )

        self.dutyOut = DutyCycleOut(0.0, use_timesync=True)

        # IR Break Beam
        self.m_irBeam = DigitalInput(0)

        # Mechanism
        # self.l_mech = Mechanism2d( 28, 28, Color8Bit(0,0,0))
        # self.l_root = self.l_mech.getRoot( "Base", 3, 3 )
        # self.l_topMotor = self.l_root.appendLigament( "TopMotor", 2, rotationsToDegrees( self.m_topMotor.get_position().value ), 2, Color8Bit(0,255,0) )

        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/Intake")
        SmartDashboard.putData( "Intake", self )

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        #self.m_logging.putNumber( "SubsystemData", 0.0 )

        # Run Subsystem: Set New State To Subsystem
        self.run()
        
        # Logging: Write Post Operation Information
        cmdName = self.getCurrentCommand().getName() if self.getCurrentCommand() != None else "None"
        self.m_logging.putString( "Command", cmdName )
        self.m_logging.putBoolean( "IRSensor", self.hasNote() )
        self.m_logging.putNumber( "Measured", self.getMeasurement() )
        self.m_logging.putNumber( "Setpoint", self.getSetpoint() )

    def simulationPeriodic(self) -> None:
        return None

    # Run the Subsystem
    def run(self) -> None:
        self.m_topMotor.set_control( self.dutyOut.with_output( self.__setpoint ) )
        self.m_bottomMotor.set_control( self.dutyOut.with_output( self.__setpoint ) )

    # Stop the Subsystem
    def stop(self) -> None:
        self.setSetpoint( IntakeConstants.STOP )

    # Set the Desired State Value
    def setSetpoint(self, speed:IntakeConstants) -> None:
        self.__setpoint = speed

    # Get the Desired Setpoint
    def getSetpoint(self) -> float:
        return self.__setpoint

    # Get the Current Speed
    def getMeasurement(self) -> float:
        return statistics.mean( [self.m_topMotor.get_duty_cycle().value, self.m_bottomMotor.get_duty_cycle().value] )

    # Set Motor Brake Mode
    def setBrake(self, brake:bool) -> None:
        mode = NeutralModeValue.BRAKE if brake else NeutralModeValue.COAST
        self.m_topMotor.setNeutralMode( mode )
        self.m_bottomMotor.setNeutralMode( mode )

    # Get the IR Beam State
    def hasNote(self) -> bool:
        return not self.m_irBeam.get()
    
