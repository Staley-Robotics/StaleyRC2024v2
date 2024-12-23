from typing import Hashable

from commands2 import SelectCommand, cmd

from commands import LauncherStart
from subsystems import Launcher, LauncherOptions
from util.Crescendo import *

class LauncherToTarget(SelectCommand):
    __prevTarget:ShredderTarget

    def __init__(
        self,
        launcherSubsystem:Launcher
    ) -> None:
        super().__init__(
            {
                ShredderTarget.SPEAKER: LauncherStart( launcherSubsystem, LauncherOptions.LONG ),
                ShredderTarget.AMP: LauncherStart( launcherSubsystem, LauncherOptions.AMP ),
                ShredderTarget.TOSS: LauncherStart( launcherSubsystem, LauncherOptions.TOSS )
            },
            Crescendo.getTarget
        )
        self._defaultCommand = cmd.none().withName( "LauncherToTarget-None" )

    def initialize(self) -> None:
        super().initialize()
        name = "None" if self._selectedCommand is None else self._selectedCommand.getName()
        self.setName( name )

    def end(self, interrupted:bool) -> None:
        self.setName( f"{self.__class__.__name__}" )
        super().end(interrupted)
