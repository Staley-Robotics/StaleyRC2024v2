from commands2 import Subsystem
from wpilib import RobotState, DriverStation, RobotBase, DigitalInput
from ntcore import NetworkTable, NetworkTableInstance
from rev import SparkMax

class IndexerSpeeds:
    #TODO PROBABLY NEED TO CHANGE THIS

    EJECT:float = -0.5 #this is negative bc eject is backwards yeye
    STOP:float = 0.0
    HANDOFF:float = 0.35

class Indexer(Subsystem):
    # Variable Declaration
    m_motor:SparkMax = None
    m_speed:float = 0.0
    m_device_id:int = None
    m_logging:NetworkTable = None

    # Initialization
    def __init__(self, device_id:int, lower_sensor_id:int, upper_sensor_id:int) -> None:
        # make motour and make no move
        self.m_deviceId = device_id
        self.m_motor = SparkMax( self.m_deviceId, SparkMax.MotorType.kBrushless )
        self.m_speed = IndexerSpeeds.STOP
        
        # Motour init bananas
        # self.m_motor.setIdleMode(self.m_motor.IdleMode.kBrake)
        #self.m_motor.IdleMode(self.m_motor.IdleMode.kBrake)
        self.m_lower_sensor = DigitalInput( lower_sensor_id )
        self.m_upper_sensor = DigitalInput( upper_sensor_id )

        self.m_logging = NetworkTableInstance.getDefault().getTable( "/Logging/Indexer" )

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.m_logging.putString( "IndexerState", "A-Ok... probably" )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        self.m_logging.putNumber( "speed", self.getSpeed() )

    # Run Indexer
    def run(self) -> None:
        """
        Runs Indexer at m_speed (technically sets the speed of the motors.... but it'll cause em to move)
        """
        self.m_motor.set(self.m_speed)

    def handoff(self):
        """
        Sets m_speed to the handoff speed
        """
        self.m_speed = IndexerSpeeds.HANDOFF

    def eject(self):
        """
        Sets m_speed to the eject speed
        """
        self.m_speed = IndexerSpeeds.EJECT

    # Stop Indexer
    def stop(self) -> None:
        self.m_speed = IndexerSpeeds.STOP

    def hasNote(self):
        """
        Returns whether or not the Indexer fully has a note (both sensors see note)
        """
        return self.m_upper_sensor.get() and self.m_upper_sensor.get()
    
    def hasHalfNote(self):
        """
        Returns
        0 if both sensors see note
        0 if no sensors see note
        1 if ONLY lower sensor sees note
        -1 if ONLY upper sensor sees note
        """
        if self.hasNote():
            return 0
        elif self.m_lower_sensor.get():
            return 1
        elif self.m_upper_sensor.get():
            return -1
        else:
            return 0

    # Set the Desired State Value
    def setSpeed(self, speed:float) -> None:
        """
        Sets m_speed to speed

        Note:
        speed must between between -1 and 1
        """
        if speed <= 1 and speed >= -1:
            self.m_speed = speed
        else:
            raise ValueError("Speed must be between -1 and 1... m_speed was not changed")

    # Get the Desired State Value
    def getSpeed(self) -> float:
        return self.m_speed
    
    # Check if Subsystem is at the Desired State
    def atSpeed(self) -> bool:
        return (self.m_motor.get() == self.m_speed)