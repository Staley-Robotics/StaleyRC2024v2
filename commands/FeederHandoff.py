from commands2 import Command

from subsystems import Feeder, FeederModes
from util.Crescendo import *

class FeederHandoff(Command):
    # Variable Declaration
    __feeder:Feeder = None
    
    # Initialization
    def __init__( self,
                  mySubsystem:Feeder
                ) -> None:
        # Command Attributes
        self.__feeder:Feeder = mySubsystem
        self.__pullBack:bool = False
        
        self.setName( "FeederHandoff" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        if self.__feeder.hasSecuredNote():
            self.cancel()
        else:
            self.__pullBack = False
            self.__feeder.setSetpoint( FeederModes.HANDOFF )

    # Periodic
    def execute(self) -> None:
        if not self.__pullBack:
            if self.__feeder.topHasNote() and not self.__feeder.bottomHasNote():
                self.__pullBack = True
                self.__feeder.setSetpoint( FeederModes.BALANCEOUT )

    # On End
    def end(self, interrupted:bool) -> None:
        Crescendo.setState( ShredderState.HOLD_INTAKE if interrupted else ShredderState.HOLD_FEEDER )
        self.__feeder.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return self.__pullBack and self.__feeder.hasSecuredNote()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False