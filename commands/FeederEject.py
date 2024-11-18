import typing

from commands2 import Command
from wpilib import Timer

from subsystems.Feeder import Feeder, FeederModes
from util.SharedConstants import SharedConstants

class FeederEject(Command):
    # Variable Declaration
    __feeder:Feeder = None
    __timer:Timer = None
    
    # Initialization
    def __init__( self,
                  mySubsystem:Feeder
                ) -> None:
        # Command Attributes
        self.__feeder:Feeder = mySubsystem
        self.setName( "FeederEject" )
        self.addRequirements( mySubsystem )
        self.__timer:Timer = Timer()

    # On Start
    def initialize(self) -> None:
        self.__feeder.setSetpoint( FeederModes.EJECT )
        self.__timer.reset()
        self.__timer.start()

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        self.__feeder.stop()
        self.__timer.stop()
        
    # Is Finished
    def isFinished(self) -> bool:
        return self.__timer.hasElapsed( SharedConstants.EJECTTIME )

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False