from commands2 import Subsystem, Command

from wpilib import RobotState, DigitalInput, SmartDashboard, RobotBase
from wpimath.system.plant import DCMotor
from wpimath.units import kSecondsPerMinute
from ntcore import NetworkTable, NetworkTableInstance
from rev import SparkMax, SparkRelativeEncoder, SparkMaxSim, SparkRelativeEncoderSim

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
    __logging:NetworkTable = None
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
        
        # Logging
        self.__logger = NetworkTableInstance.getDefault().getTable("/Logging/Feeder")
        self.__measured = NetworkTableInstance.getDefault().getTable("/RealOutputs/Feeder")

        # Dashboards
        SmartDashboard.putData( "Feeder", self )

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.__logger.putNumber( "MotorInput", self.__motor.get() )
        self.__logger.putNumber( "MotorOutput", self.__motor.getAppliedOutput() )
        self.__logger.putNumber( "MotorPosition_r", self.__motorEncoder.getPosition() )
        self.__logger.putNumber( "MotorVelocity_rpm", self.__motorEncoder.getVelocity() )
        self.__logger.putBoolean( "SensorTop", self.__topIrBeam.get() )
        self.__logger.putBoolean( "SensorBottom", self.__bottomIrBeam.get() )

        # Run Subsystem: Set New State To Subsystem
        self.run()
      
        # Logging: Write Post Operation Information
        cmdName = self.getCurrentCommand().getName() if self.getCurrentCommand() != None else "None"
        self.__measured.putString( "Command", cmdName )
        self.__measured.putNumber( "Setpoint", self.getSetpoint() )
        self.__measured.putBoolean( "HasNote", self.hasSecuredNote() )
        self.__measured.putBoolean( "HasNoteTop", self.topHasNote() )
        self.__measured.putBoolean( "HasNoteBottom", self.bottomHasNote() )
        
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