from commands2 import Command
from subsystems.Intake import Intake, IntakeConstants

class IntakeHandoff(Command):
    # Variable Declaration
    m_intake:Intake = None
    
    # Initialization
    def __init__( self,
                  mySubsystem:Intake
                ) -> None:
        # Command Attributes
        self.m_intake:Intake = mySubsystem
        self.setName( "IntakeHandoff" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        self.m_intake.setBrake( False )

    # Periodic
    def execute(self) -> None:
        self.m_intake.setSetpoint( IntakeConstants.HANDOFF )

    # On End
    def end(self, interrupted:bool) -> None:
        self.m_intake.setSetpoint( IntakeConstants.STOP )

    # Is Finished
    def isFinished(self) -> bool:
        return not self.m_intake.isSensorTripped()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False