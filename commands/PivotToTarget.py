from typing import Hashable

from commands2 import SelectCommand, cmd

from commands import PivotToPosition
from subsystems import Pivot, PivotPositions
from util.Crescendo import *

class PivotToTarget(SelectCommand):
    def __init__(
        self,
        pivotSubsystem:Pivot
    ) -> None:
        super().__init__(
            {
                ShredderTarget.NONE: PivotToPosition( pivotSubsystem, PivotPositions.FLAT ),
                ShredderTarget.SPEAKER: PivotToPosition( pivotSubsystem, PivotPositions.SPEAKER ),
                ShredderTarget.AMP: PivotToPosition( pivotSubsystem, PivotPositions.AMP ),
                ShredderTarget.TOSS: PivotToPosition( pivotSubsystem, PivotPositions.TOSS )
            },
            Crescendo.getTarget
        )
        self._defaultCommand = cmd.none().withName("PivotToTarget-None")

    def initialize(self) -> None:
        super().initialize()
        self.__prevTarget = self.__getCurrentTarget()
        name = "None" if self._selectedCommand is None else self._selectedCommand.getName()
        self.setName( name )

    def execute(self) -> None:
        self.__prevTarget = self.__getCurrentTarget()
        super().execute()

    def isFinished(self) -> bool:
        changedState = self.__hasStateChanged()
        isFinished = super().isFinished()
        return changedState or isFinished

    def end(self, interrupted:bool) -> None:
        self.setName( f"{self.__class__.__name__}" )
        super().end(interrupted)

    def __hasStateChanged(self) -> bool:
        return self.__getPreviousTarget() != self.__getCurrentTarget()

    def __getPreviousTarget(self) -> Hashable:
        return self.__prevTarget
    
    def __getCurrentTarget(self) -> Hashable:
        return self._selector()