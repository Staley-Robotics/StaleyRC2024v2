import statistics, typing, threading

from commands2 import Subsystem, cmd, Command

from wpilib import SmartDashboard, RobotState, Mechanism2d, Color8Bit, RobotController, DigitalInput
from wpilib.shuffleboard import Shuffleboard
from wpimath.system.plant import DCMotor
from wpimath.units import rotationsToDegrees, radiansToRotations
from ntcore import NetworkTable, NetworkTableInstance

from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue

class IntakeOptions:
    EJECT:float = -1.0
    STOP:float = 0.0
    HANDOFF:float = 0.5
    PICKUP:float = 0.5

class IntakeConstants:
    class FalconSim:
        kMaxRps:float = radiansToRotations( DCMotor.falcon500FOC(1).freeSpeed )

class Intake(Subsystem):
    # Variable Declaration
    __topMotor:TalonFX = None
    __bottomMotor:TalonFX = None
    __irBeam:DigitalInput = None
    __logger:NetworkTable = None

    __setpoint:float = 0.0

    # Initialization
    def __init__(self) -> None:
        # Top Motor
        self.__topMotorCfg = TalonFXConfiguration()
        self.__topMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.__topMotorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        self.__topMotor = TalonFX(20, "canivore1")
        self.__topMotor.configurator.apply( self.__topMotorCfg )

        # Bottom Motor
        self.__bottomMotorCfg = TalonFXConfiguration()
        self.__bottomMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.__bottomMotorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        self.__bottomMotor = TalonFX(21, "canivore1")
        self.__bottomMotor.configurator.apply( self.__bottomMotorCfg )

        # IR Break Beam
        self.__irBeam = DigitalInput(0)

        # Mechanism
        # self.l_mech = Mechanism2d( 28, 28, Color8Bit(0,0,0))
        # self.l_root = self.l_mech.getRoot( "Base", 3, 3 )
        # self.l_topMotor = self.l_root.appendLigament( "TopMotor", 2, rotationsToDegrees( self.__topMotor.get_position().value ), 2, Color8Bit(0,255,0) )

        # Logging
        Shuffleboard.getTab( "Intake" ).add( "Intake", self )
        self.__logger = NetworkTableInstance.getDefault().getTable("/Logging/Intake")
        self.__outputs = NetworkTableInstance.getDefault().getTable("/RealOutputs/Intake")

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.__logger.putNumber( "MotorTopInput", self.__topMotor.get() )
        self.__logger.putNumber( "MotorTopOutput", self.__topMotor.get_motor_voltage().value )
        self.__logger.putNumber( "MotorTopPosition_r", self.__topMotor.get_position().value )
        self.__logger.putNumber( "MotorTopVelocity_rps", self.__topMotor.get_velocity().value )
        self.__logger.putNumber( "MotorBottomInput", self.__bottomMotor.get() )
        self.__logger.putNumber( "MotorBottomOutput", self.__bottomMotor.get_motor_voltage().value )
        self.__logger.putNumber( "MotorBottomPosition_r", self.__bottomMotor.get_position().value )
        self.__logger.putNumber( "MotorBottomVelocity_rps", self.__bottomMotor.get_velocity().value )
        self.__logger.putBoolean( "Sensor", self.__irBeam.get() )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        self.run()
        
        # Logging: Write Post Operation Information
        cmdName = self.getCurrentCommand().getName() if self.getCurrentCommand() != None else "None"
        self.__outputs.putString( "Command", cmdName )
        self.__outputs.putNumber( "Setpoint", self.getSetpoint() )
        self.__outputs.putBoolean( "HasNote", self.hasNote() )

    def simulationPeriodic(self) -> None:
        # Motor Position and Velocity
        velocity = IntakeConstants.FalconSim.kMaxRps * self.__topMotor.get()

        self.__topMotor.sim_state.set_rotor_velocity( velocity )
        self.__topMotor.sim_state.add_rotor_position( velocity * 0.02 )

        self.__bottomMotor.sim_state.set_rotor_velocity( velocity )
        self.__bottomMotor.sim_state.add_rotor_position( velocity * 0.02 )

    # Run the Subsystem
    def run(self) -> None:
        self.__topMotor.set( self.__setpoint )
        self.__bottomMotor.set( self.__setpoint )

    # Stop the Subsystem
    def stop(self) -> None:
        self.setSetpoint( IntakeOptions.STOP )

    # Set the Desired State Value
    def setSetpoint(self, speed:float) -> None:
        self.__setpoint = speed

    # Get the Desired Setpoint
    def getSetpoint(self) -> float:
        return self.__setpoint

    # Get the Current Speed
    def getMeasurement(self) -> float:
        return statistics.mean( [self.__topMotor.get_duty_cycle().value, self.__bottomMotor.get_duty_cycle().value] )

    # Set Motor Brake Mode (Uses Multi-Threaded Processing to prevent periodic cycle skip)
    def setBrake(self, brake:bool) -> None:
        def changeBrake( brakeSetting:bool ):
            mode = NeutralModeValue.BRAKE if brakeSetting else NeutralModeValue.COAST
            self.__topMotor.setNeutralMode( mode, 0.1 )
            self.__bottomMotor.setNeutralMode( mode, 0.1 )

        threading.Thread( target=lambda: changeBrake(brake) ).start()

    # Get the IR Beam State
    def hasNote(self) -> bool:
        return not self.__irBeam.get()
    
