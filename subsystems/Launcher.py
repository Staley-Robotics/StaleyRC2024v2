from commands2 import PIDSubsystem

from wpilib import RobotState
from wpilib.shuffleboard import Shuffleboard
from wpilib.simulation import DCMotorSim
from wpimath.controller import PIDController, SimpleMotorFeedforwardRadians
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations, rotationsToRadians

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import InvertedValue
from phoenix6.controls import VoltageOut

from util import FalconLogger

class LauncherOptions:
    AMP:float = 25.0
    TOSS:float = 50.0
    SHORT:float = 60.0
    MEDIUM:float = 80.0
    LONG:float = 100.0
    STOP:float = 0.0
    RECEIVE:float = -10.0

class LauncherConstants:
    kP:float = 0.0
    kI:float = 0.0
    kD:float = 0.0
    kS:float = 0.0
    kV:float = 0.018865
    kTolerance:float = 5.0

    leftOutputPercent:float = 0.80 # Percentage

    class FalconSim:
        kMaxRps = radiansToRotations( DCMotor.falcon500FOC(1).freeSpeed )

class Launcher(PIDSubsystem):
    # Variable Declaration
    __leftMotor:TalonFX = None
    __rightMotor:TalonFX = None
    __feedFwd:SimpleMotorFeedforwardRadians = None

    # Initialization
    def __init__(self) -> None:
        # Left Motor
        leftMotorCfg = TalonFXConfiguration()
        leftMotorCfg.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.__leftMotor = TalonFX(23, "canivore1")

        # Right Motor
        rightMotorCfg = TalonFXConfiguration()
        rightMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.__rightMotor = TalonFX(24, "canivore1")

        # Simulation Motors
        simLeftMotor = DCMotor.falcon500FOC(1)
        self.__simLeftMotor = DCMotorSim( LinearSystemId.DCMotorSystem( simLeftMotor, 0.001, 1.0 ), simLeftMotor )
        simRightMotor = DCMotor.falcon500FOC(1)
        self.__simRightMotor = DCMotorSim( LinearSystemId.DCMotorSystem( simRightMotor, 0.001, 1.0 ), simRightMotor )

        # PID Controller
        pidController = PIDController( LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD )
        pidController.setTolerance( LauncherConstants.kTolerance )
        self.__feedFwd = SimpleMotorFeedforwardRadians( LauncherConstants.kS, LauncherConstants.kV )

        # PIDSubsystem Setup
        super().__init__(
            pidController,
            0.0
        )
        self.enable()

        # Dashboard
        Shuffleboard.getTab( "Launcher" ).add( "Launcher", self )
        Shuffleboard.getTab( "Launcher" ).add( "LauncherPid", self.getController() )

    # Periodic Loop
    def periodic(self) -> None:
        # Input Logging
        FalconLogger.logInput( "Launcher/Left/MotorInput", self.__leftMotor.get() )
        FalconLogger.logInput( "Launcher/Left/MotorOutput", self.__leftMotor.get_motor_voltage().value )
        FalconLogger.logInput( "Launcher/Left/MotorPosition_r", self.__leftMotor.get_position().value )
        FalconLogger.logInput( "Launcher/Left/MotorVelocity_rps", self.__leftMotor.get_velocity().value )
        FalconLogger.logInput( "Launcher/Right/MotorInput", self.__rightMotor.get() )
        FalconLogger.logInput( "Launcher/Right/MotorOutput", self.__rightMotor.get_motor_voltage().value )
        FalconLogger.logInput( "Launcher/Right/MotorPosition_r", self.__rightMotor.get_position().value )
        FalconLogger.logInput( "Launcher/Right/MotorVelocity_rps", self.__rightMotor.get_velocity().value )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        super().periodic()
        
        # Output Logging
        FalconLogger.logOutput( "ActualSpeed", self.getMeasurement() )
        FalconLogger.logOutput( "TargetSpeed", self.getSetpoint() )
        FalconLogger.logOutput( "AtSetpoint", self.atSetpoint() )

    # Simulation Periodic
    def simulationPeriodic(self):      
        # Iterate Sim Motor
        leftVoltage = self.__leftMotor.sim_state.motor_voltage
        self.__simLeftMotor.setInputVoltage( leftVoltage )
        self.__simLeftMotor.update(0.02)
        self.__leftMotor.sim_state.set_raw_rotor_position( radiansToRotations( self.__simLeftMotor.getAngularPosition() ) )
        self.__leftMotor.sim_state.set_rotor_velocity( radiansToRotations( self.__simLeftMotor.getAngularAcceleration() ) )
        self.__leftMotor.sim_state.set_rotor_acceleration( radiansToRotations( self.__simLeftMotor.getAngularPosition() ) )

        rightVoltage = self.__rightMotor.sim_state.motor_voltage
        self.__simRightMotor.setInputVoltage( rightVoltage )
        self.__simRightMotor.update(0.02)
        self.__rightMotor.sim_state.set_raw_rotor_position( radiansToRotations( self.__simRightMotor.getAngularPosition() ) )
        self.__rightMotor.sim_state.set_rotor_velocity( radiansToRotations( self.__simRightMotor.getAngularAcceleration() ) )
        self.__rightMotor.sim_state.set_rotor_acceleration( radiansToRotations( self.__simRightMotor.getAngularPosition() ) )

        # Motor Position and Velocity
        # right_velocity = LauncherConstants.FalconSim.kMaxRps * (self.__rightMotor.sim_state.motor_voltage / 12.0 )
        # self.__rightMotor.sim_state.set_rotor_velocity( right_velocity )
        # self.__rightMotor.sim_state.add_rotor_position( right_velocity * 0.02 )

        # left_velocity = LauncherConstants.FalconSim.kMaxRps * (self.__leftMotor.sim_state.motor_voltage / 12.0 )
        # self.__leftMotor.sim_state.set_rotor_velocity( left_velocity )
        # self.__leftMotor.sim_state.add_rotor_position( left_velocity * 0.02 )

        #if self.atSetpoint() and self.getMeasurement() != 0.0: print( "Target Achieved!" )

    # Stop the Subsystem
    def stop(self) -> None:
        self.setSetpoint( LauncherOptions.STOP )

    # Set the Desired State Value
    def setSetpoint(self, setpoint:float) -> None:
        # Limts the specific range (Protects mechanism)
        setpoint = min( max( setpoint, -LauncherConstants.FalconSim.kMaxRps ), LauncherConstants.FalconSim.kMaxRps )
        return super().setSetpoint( setpoint )

    # Run the Subsystem
    def useOutput(self, output:float, setpoint:float) -> None:
        # Placeholder: Feed Forward
        ffVolts = self.__feedFwd.calculate( rotationsToRadians( setpoint ) )
        motorOutput = output + ffVolts
        self.__rightMotor.set_control( VoltageOut(motorOutput) )
        self.__leftMotor.set_control( VoltageOut( motorOutput * LauncherConstants.leftOutputPercent ) )

    def getMeasurement(self) -> float:
        return self.__rightMotor.get_velocity().value
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        return self._controller.atSetpoint()
    
    def hasLaunched(self) -> bool:
        return False