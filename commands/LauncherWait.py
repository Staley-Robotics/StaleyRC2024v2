from commands2 import Command

from subsystems import Launcher
from util.Crescendo import *

class LauncherWait(Command):
    # Initialization
    def __init__( self,
                  mySubsystem:Launcher
                ) -> None:
        # Command Attributes
        self.__launcher:Launcher = mySubsystem

        self.setName( f"LauncherWait" )
        self.addRequirements( mySubsystem )

    # On End
    def end(self, interrupted:bool) -> None:
        Crescendo.setState( ShredderState.PREPARE_TO_SHOOT if interrupted else ShredderState.READY_TO_SHOOT )
        return None #self.__launcher.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False