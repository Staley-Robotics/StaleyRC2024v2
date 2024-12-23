from commands2 import Command

from subsystems import Launcher
from util.Crescendo import *

class LauncherStart(Command):
    # Initialization
    def __init__( self,
                  mySubsystem:Launcher,
                  mySpeed: float = 0.0
                ) -> None:
        # Command Attributes
        self.__launcher:Launcher = mySubsystem
        self.__speed:float = mySpeed

        self.setName( f"LauncherStart({self.__speed})" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        if Crescendo.getState() == ShredderState.HOLD_FEEDER:
            Crescendo.setState( ShredderState.PREPARE_TO_SHOOT )
        self.__launcher.setSetpoint( self.__speed )

    # Periodic
    def execute(self) -> None:
        #Crescendo.setState( ShredderState.PREPARE_TO_SHOOT )
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        if not interrupted:
            if Crescendo.getState() == ShredderState.PREPARE_TO_SHOOT:
                Crescendo.setState( ShredderState.READY_TO_SHOOT )
        return None

    # Is Finished
    def isFinished(self) -> bool:
        return self.__launcher.atSetpoint()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False