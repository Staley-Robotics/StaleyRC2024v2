from commands2 import Command

from subsystems import Launcher, LauncherOptions

class LauncherStop(Command):
    # Initialization
    def __init__( self,
                  mySubsystem:Launcher
                ) -> None:
        # Command Attributes
        self.__launcher:Launcher = mySubsystem

        self.setName( f"LauncherStop" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        self.__launcher.setSetpoint( LauncherOptions.STOP )

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        self.__launcher.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return True # self.__launcher.hasLaunched()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False