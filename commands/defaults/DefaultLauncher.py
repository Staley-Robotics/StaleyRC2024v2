# Python Imports
from typing import Callable

# WPI Imports
from commands2 import * #cmd

# Team Command Based Imports
from commands import *
from subsystems import Launcher

# Team Utility Imports
from util import DefaultCommand
from util.Crescendo import *

class DefaultLauncher(DefaultCommand):
    __autoStart:bool = True

    def __init__(
        self,
        launcherSubsystem:Launcher
    ):
        super().__init__(
            {
                #ShredderState.DEFAULT: LauncherStop( launcherSubsystem ),
                ShredderState.UNBALANCED: LauncherStop( launcherSubsystem ),
                ShredderState.HOLD_FEEDER: ConditionalCommand(
                    LauncherToTarget( launcherSubsystem ),
                    LauncherStop( launcherSubsystem ), 
                    self.getAutoStart
                ).withName( "LauncherWait" ),
                ShredderState.PREPARE_TO_SHOOT: LauncherToTarget( launcherSubsystem ),
                ShredderState.READY_TO_SHOOT: LauncherToTarget( launcherSubsystem ),
                ShredderState.SHOT_COMPLETE: LauncherStop( launcherSubsystem )
            },
            Crescendo.getState
        )
    
    def getAutoStart(self):
        return self.__autoStart

    def setAutoStart(self, enable:bool) -> None:
        self.__autoStart = enable
        print( f"AutoStart Launcher: {self.__autoStart}" )

    def toggleAutoStart(self):
        self.setAutoStart( not self.getAutoStart() )