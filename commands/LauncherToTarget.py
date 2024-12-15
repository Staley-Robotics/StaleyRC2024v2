from typing import Hashable

from commands2 import SelectCommand, cmd

from commands import LauncherStart
from subsystems import Launcher, LauncherOptions
from util import ShredderTarget, Crescendo, ShredderState

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
        self.__prevTarget = self.__getCurrentTarget()
        name = "None" if self._selectedCommand is None else self._selectedCommand.getName()
        self.setName( name )

    def execute(self) -> None:
        super().execute()

    def isFinished(self) -> bool:
        changedTarget = self.__hasTargetChanged()
        if changedTarget:
            Crescendo.setState( ShredderState.PREPARE_TO_SHOOT )
        isFinished = super().isFinished()
        return changedTarget or isFinished

    def end(self, interrupted:bool) -> None:
        self.setName( f"{self.__class__.__name__}" )
        super().end(interrupted)

    def __hasTargetChanged(self) -> bool:
        return self.__getPreviousTarget() != self.__getCurrentTarget()

    def __getPreviousTarget(self) -> Hashable:
        return self.__prevTarget
    
    def __getCurrentTarget(self) -> Hashable:
        return self._selector()