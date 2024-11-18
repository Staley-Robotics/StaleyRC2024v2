from commands2 import Command
from subsystems import Intake, IntakeOptions

class IntakePickup(Command):
    # Initialization
    def __init__( self,
                  mySubsystem:Intake
                ) -> None:
        # Command Attributes
        self.__intake:Intake = mySubsystem
        
        self.setName( "IntakePickup" )
        self.addRequirements( mySubsystem )
        
    # On Start
    def initialize(self) -> None:
        self.__intake.setBrake( True )

    # Periodic
    def execute(self) -> None:
        self.__intake.setSetpoint( IntakeOptions.PICKUP )

    # On End
    def end(self, interrupted:bool) -> None:
        self.__intake.setSetpoint( IntakeOptions.STOP )

    # Is Finished
    def isFinished(self) -> bool:
        return self.__intake.hasNote()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False