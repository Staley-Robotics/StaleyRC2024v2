import typing

from commands2 import Command, Subsystem
from subsystems.Launcher import Launcher, LauncherOptions

class LauncherStart(Command):
    # Variable Declaration
    __launcher:Launcher = None
    __speed:float = 0.0
    
    # Initialization
    def __init__( self,
                  mySubsystem:Launcher,
                  mySpeed: float = 0.0
                ) -> None:
        # Command Attributes
        self.__launcher:Launcher = mySubsystem
        self.__speed = mySpeed
        self.setName( f"Launcher_{self.__speed}" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        self.__launcher.setSetpoint( self.__speed )

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        self.__launcher.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return self.__launcher.hasLaunched()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False