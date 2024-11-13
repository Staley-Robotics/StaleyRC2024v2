from commands2 import Subsystem, Command

from wpilib import RobotState, DigitalInput, SmartDashboard
from ntcore import NetworkTable, NetworkTableInstance
from rev import SparkMax, SparkRelativeEncoder, SparkRelativeEncoderSim

class FeederModes:
    STOP:float = 0.0
    BALANCEIN:float = 0.3
    BALANCEOUT:float = -0.3
    LAUNCH:float = 0.7
    HANDOFF:float = 0.5
    EJECT:float = -1.0
    MAX:float = 1.0
    MIN:float = -1.0

class FeederBalance(Command):
    def __init__(self, intake:Subsystem):
        self.setName( "FeederBalance" )
        self.addRequirements( intake )
        self.__feeder:Feeder = intake

    def initialize(self):
        if self.__feeder.topHasNote():
            self.__feeder.setSetpoint( FeederModes.BALANCEOUT )
        elif self.__feeder.bottomHasNote():
            self.__feeder.setSetpoint( FeederModes.BALANCEIN )
        else:
            self.__feeder.setSetpoint( FeederModes.STOP )

    def end(self, interrupted):
        self.__feeder.stop()
    
    def isFinished(self):
        return self.__feeder.bottomHasNote() == self.__feeder.topHasNote()

class Feeder(Subsystem):
    # Variable Declaration
    __motor:SparkMax = None
    __setpoint:int = 0.0
    __bottomIrBeam:DigitalInput = None
    __topIrBeam:DigitalInput = None
    __logging:NetworkTable = None

    # Initialization
    def __init__(self) -> None:
        # Motor Settings
        self.__motor = SparkMax( 22, SparkMax.MotorType.kBrushless )
        self.__motorEncoder:SparkRelativeEncoder = self.__motor.getEncoder()
        
        # IR Break Beams
        self.__bottomIrBeam:DigitalInput = DigitalInput(1)
        self.__topIrBeam:DigitalInput = DigitalInput(2)
        
        # Internal Commands
        self.balanceCmd = FeederBalance(self)

        # Logging
        self.__logging = NetworkTableInstance.getDefault().getTable("/Logging/Feeder")

        # Dashboards
        SmartDashboard.putData( "Intake", self )

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.__logging.putNumber( "aMotorPositionRot", self.__motorEncoder.getPosition() )
        self.__logging.putNumber( "aMotorVelocityRPM", self.__motorEncoder.getVelocity() )
        self.__logging.putBoolean( "bSensorTop", self.topHasNote() )
        self.__logging.putBoolean( "bSensorBottom", self.bottomHasNote() )

        # Run Subsystem: Set New State To Subsystem
        self.run()
      
        # Logging: Write Post Operation Information
        self.__logging.putNumber( "Setpoint", self.getSetpoint() )
        self.__logging.putNumber( "Measured", self.getMeasurement() )

        # Misaligned Note Correction
        if self.getCurrentCommand() == None:
            if self.bottomHasNote() != self.topHasNote():
                self.balanceCmd.schedule()

    def simulationPeriodic(self) -> None:
        pass

    # Run the Subsystem
    def run(self) -> None:
        self.__motor.set( self.__setpoint )

    # Stop the Subsystem
    def stop(self) -> None:
        self.setSetpoint( FeederModes.STOP )

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