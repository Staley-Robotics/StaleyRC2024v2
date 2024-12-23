# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd, SelectCommand

# Team Command Based Imports
from commands import *
from subsystems import Pivot, PivotPositions

# Team Utility Imports
from util import *
from util.Crescendo import *

class DefaultPivot(DefaultCommand):
    def __init__(
        self,
        pivotSubsystem:Pivot
    ) -> None:
        super().__init__(
            {
                ShredderState.PICKUP: PivotToPosition( pivotSubsystem, PivotPositions.HANDOFF ),
                ShredderState.HOLD_INTAKE: PivotToPosition( pivotSubsystem, PivotPositions.HANDOFF ),
                ShredderState.WAIT_FOR_PIVOT:  PivotToPosition( pivotSubsystem, PivotPositions.HANDOFF ),
                ShredderState.HANDOFF: PivotToPosition( pivotSubsystem, PivotPositions.HANDOFF ),
                ShredderState.HOLD_FEEDER: PivotToTarget( pivotSubsystem ),
                ShredderState.PREPARE_TO_SHOOT: PivotToTarget( pivotSubsystem ),
                ShredderState.READY_TO_SHOOT: PivotToTarget( pivotSubsystem ),
                ShredderState.SHOOTING: PivotToTarget( pivotSubsystem ),
                ShredderState.SHOT_COMPLETE: PivotToPosition( pivotSubsystem, PivotPositions.HANDOFF )
            },
            Crescendo.getState
        )