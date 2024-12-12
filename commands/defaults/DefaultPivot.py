# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import Pivot

# Team Utility Imports
from gamestates.ShredderState import ShredderState
from util.DefaultCommand import DefaultCommand

class DefaultPivot(DefaultCommand):
    def __init__(
        self,
        pivotSubsystem:Pivot,
        gameState:Callable[[],ShredderState] = lambda: None,
    ) -> None:
        super().__init__(
            {
                ShredderState.DoNothing: cmd.run( lambda: None, pivotSubsystem )
            },
            gameState
        )