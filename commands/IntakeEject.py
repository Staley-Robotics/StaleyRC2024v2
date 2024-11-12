from commands2 import Command
from wpilib import Timer
from subsystems.Intake import Intake, IntakeConstants

class IntakeEject(Command):
    # Variable Declaration
    m_intake:Intake = None
    m_timer:Timer = None

    # Constants
    m_timerElapsed:float = 5.0
    
    # Initialization
    def __init__( self,
                  mySubsystem:Intake
                ) -> None:
        # Command Attributes
        self.m_intake:Intake = mySubsystem
        self.setName( "IntakeEject" )
        self.addRequirements( mySubsystem )
        self.m_timer:Timer = Timer()

    # On Start
    def initialize(self) -> None:
        self.m_intake.setBrake( False )
        self.m_timer.reset()
        self.m_timer.start()

    # Periodic
    def execute(self) -> None:
        self.m_intake.setSetpoint( IntakeConstants.EJECT )

    # On End
    def end(self, interrupted:bool) -> None:
        self.m_intake.setSetpoint( IntakeConstants.STOP )

    # Is Finished
    def isFinished(self) -> bool:
        return self.m_timer.hasElapsed( self.m_timerElapsed )

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False