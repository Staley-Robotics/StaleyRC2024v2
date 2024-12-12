# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import Launcher

# Team Utility Imports
from gamestates.ShredderState import ShredderState
from util.DefaultCommand import DefaultCommand

class DefaultLauncher(DefaultCommand):
    def __init__(
        self,
        launcherSubsystem:Launcher,
        gameState:Callable[[],ShredderState] = lambda: None,
    ):
        super().__init__(
            {
                ShredderState.DoNothing: cmd.run( lambda: None, launcherSubsystem )
            },
            gameState
        )