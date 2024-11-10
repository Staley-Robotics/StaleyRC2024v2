from commands2 import Subsystem
from wpilib import RobotState
from ntcore import NetworkTable, NetworkTableInstance

from subsystems.Intake import Intake
from subsystems.Feeder import Feeder
from subsystems.Pivot import Pivot
from subsystems.Launcher import Launcher

class NoteFactory(Subsystem):
    # Variable Declaration
    m_value:float = 0.0
    m_system:int = None
    m_logging:NetworkTable = None

    m_intake:Intake = None
    m_feeder:Feeder = None
    m_pivot:Pivot = None
    m_launcher:Launcher = None

    # Initialization
    def __init__(self, sysId:int) -> None:
        self.setName( "NoteFactory" )
        
        self.intake = Intake()
        self.feeder = Feeder()
        self.pivot = Pivot()
        self.launcher = Launcher()

        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/NoteFactory")

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.m_logging.putNumber( "SubsystemData", 0.0 )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        self.m_logging.putNumber( "Setpoint", self.getSetpoint() )
        self.m_logging.putNumber( "Measured", self.m_system )

    # Run the Subsystem
    def run(self) -> None:
        pass

    # Stop the Subsystem
    def stop(self) -> None:
        pass

    # Set the Desired State Value
    def setSetpoint(self, value:float) -> None:
        self.m_value = value

    # Get the Desired State Value
    def getSetpoint(self) -> float:
        return self.m_value
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        return False