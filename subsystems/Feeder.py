from commands2 import Subsystem, Command

from wpilib import RobotState, DigitalInput, RobotBase
from wpilib.shuffleboard import Shuffleboard
from wpimath.system.plant import DCMotor
from wpimath.units import kSecondsPerMinute
from rev import SparkMax, SparkRelativeEncoder, SparkMaxSim

from util import FalconLogger

class FeederModes:
    STOP:float = 0.0
    BALANCEIN:float = 0.3
    BALANCEOUT:float = -0.3
    LAUNCH:float = 0.7
    HANDOFF:float = 0.5
    EJECT:float = -1.0
    MAX:float = 1.0
    MIN:float = -1.0

class FeederConstants:
    class NeoSim:
        kMaxRpm = DCMotor.NEO(1).freeSpeed * kSecondsPerMinute

class Feeder(Subsystem):
    # Variable Declaration
    __motor:SparkMax = None
    __setpoint:int = 0.0
    __bottomIrBeam:DigitalInput = None
    __topIrBeam:DigitalInput = None
    __balanceCmd:Command = None

    # Initialization
    def __init__(self) -> None:
        # Motor Settings
        self.__motor = SparkMax( 22, SparkMax.MotorType.kBrushless )
        self.__motorEncoder:SparkRelativeEncoder = self.__motor.getEncoder()

        # Simulation
        if RobotBase.isSimulation():
            self.__motorSim = SparkMaxSim( self.__motor, DCMotor.NEO(1) )
            self.__motorSim.getAbsoluteEncoderSim().setPositionConversionFactor(1)
            self.__motorSim.getAbsoluteEncoderSim().setVelocityConversionFactor(1)
            self.__motorSim.getRelativeEncoderSim().setPositionConversionFactor(1)
            self.__motorSim.getRelativeEncoderSim().setVelocityConversionFactor(1)
        
        # IR Break Beams
        self.__bottomIrBeam:DigitalInput = DigitalInput(1)
        self.__topIrBeam:DigitalInput = DigitalInput(2)
        
        # Dashboards
        Shuffleboard.getTab( "Feeder" ).add( "Feeder", self )

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        FalconLogger.logInput( "Feeder/MotorInput", self.__motor.get() )
        FalconLogger.logInput( "Feeder/MotorOutput", self.__motor.getAppliedOutput() )
        FalconLogger.logInput( "Feeder/MotorPosition_r", self.__motorEncoder.getPosition() )
        FalconLogger.logInput( "Feeder/MotorVelocity_rpm", self.__motorEncoder.getVelocity() )
        FalconLogger.logInput( "Feeder/SensorTop", self.__topIrBeam.get() )
        FalconLogger.logInput( "Feeder/SensorBottom", self.__bottomIrBeam.get() )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        self.run()
      
        # Logging: Write Post Operation Information
        cmdName = self.getCurrentCommand().getName() if self.getCurrentCommand() != None else "None"
        FalconLogger.logOutput( "Feeder/Command", cmdName )
        FalconLogger.logOutput( "Feeder/Setpoint", self.getSetpoint() )
        FalconLogger.logOutput( "Feeder/HasNote", self.hasSecuredNote() )
        FalconLogger.logOutput( "Feeder/HasNoteTop", self.topHasNote() )
        FalconLogger.logOutput( "Feeder/HasNoteBottom", self.bottomHasNote() )
        
        # Misaligned Note Correction
        if self.getCurrentCommand() == None and self.__balanceCmd != None:
            if self.bottomHasNote() != self.topHasNote():
                self.__balanceCmd.schedule()

    def simulationPeriodic(self) -> None:
        # Motor Position and Velocity
        driveRpm = FeederConstants.NeoSim.kMaxRpm * self.__motor.get()
        self.__motorSim.getRelativeEncoderSim().iterate( driveRpm, 0.02 )
        self.__motorSim.getRelativeEncoderSim().setVelocity( driveRpm )
        self.__motorSim.getAbsoluteEncoderSim().setVelocity( driveRpm )
        self.__motorSim.getAbsoluteEncoderSim().setPosition( self.__motorSim.getRelativeEncoderSim().getPosition() % 1 )

    # Run the Subsystem
    def run(self) -> None:
        self.__motor.set( self.__setpoint )

    # Stop the Subsystem
    def stop(self) -> None:
        self.setSetpoint( FeederModes.STOP )

    def addBalanceCommand(self, cmd:Command) -> None:
        self.__balanceCmd = cmd

    def removeBalanceCommand(self) -> None:
        self.__balanceCmd = None

    # Set the Desired State Value
    def setSetpoint(self, value:float) -> None:
        value = min( max( value, FeederModes.MIN ), FeederModes.MAX )
        self.__setpoint = value

    # Get the Desired State Value
    def getSetpoint(self) -> float:
        return self.__setpoint
    
    def getMeasurement(self) -> float:
        return self.__motor.get()

    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        return False
    
    # Bottom Sensor
    def bottomHasNote(self) -> bool:
        return not self.__bottomIrBeam.get()
    
    # Top Sensor
    def topHasNote(self) -> bool:
        return not self.__topIrBeam.get()
    
    # Note Secured in Both Sensors
    def hasSecuredNote(self) -> bool:
        return self.topHasNote() and self.bottomHasNote()