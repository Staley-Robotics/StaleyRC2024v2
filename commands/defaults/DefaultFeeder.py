# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import Feeder

# Team Utility Imports
from gamestates.ShredderState import ShredderState
from util.DefaultCommand import DefaultCommand

class DefaultFeeder(DefaultCommand):
    def __init__(
        self,
        feederSubsystem:Feeder,
        gameState:Callable[[],ShredderState] = lambda: None,
    ):
        super().__init__(
            {
                ShredderState.DoNothing: cmd.run( lambda: None, feederSubsystem )
            },
            gameState
        )