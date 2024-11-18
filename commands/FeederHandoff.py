import typing

from commands2 import Command

from subsystems.Feeder import Feeder, FeederModes

class FeederHandoff(Command):
    # Variable Declaration
    __feeder:Feeder = None
    
    # Initialization
    def __init__( self,
                  mySubsystem:Feeder
                ) -> None:
        # Command Attributes
        self.__feeder:Feeder = mySubsystem
        self.setName( "FeederHandoff" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        if self.__feeder.hasSecuredNote():
            self.cancel()
        else:
            self.__feeder.setSetpoint( FeederModes.HANDOFF )

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        self.__feeder.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return self.__feeder.topHasNote() and not self.__feeder.bottomHasNote()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False