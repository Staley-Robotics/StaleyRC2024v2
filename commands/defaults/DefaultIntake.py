# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import Intake

# Team Utility Imports
from gamestates.ShredderState import ShredderState
from util.DefaultCommand import DefaultCommand

class DefaultIntake(DefaultCommand):
    def __init__(
        self,
        intakeSubsystem:Intake,
        gameState:Callable[[],ShredderState] = lambda: None,
    ):
        super().__init__(
            {
                ShredderState.DoNothing: cmd.run( lambda: None, intakeSubsystem )
            },
            gameState
        )