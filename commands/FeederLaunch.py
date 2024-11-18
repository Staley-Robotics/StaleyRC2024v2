import typing

from commands2 import Command

from subsystems import Feeder, FeederModes

class FeederLaunch(Command):
    # Variable Declaration
    __feeder:Feeder = None
    
    # Initialization
    def __init__( self,
                  mySubsystem:Feeder
                ) -> None:
        # Command Attributes
        self.__feeder:Feeder = mySubsystem
        
        self.setName( "FeederLaunch" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        if not self.__feeder.hasSecuredNote():
            self.cancel()
        else:
            self.__feeder.setSetpoint( FeederModes.LAUNCH )

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        self.__feeder.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return not self.__feeder.bottomHasNote() and not self.__feeder.topHasNote()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False