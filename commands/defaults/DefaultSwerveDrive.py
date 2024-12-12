# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import SwerveDrive

# Team Utility Imports
from gamestates.ShredderState import ShredderState
from util.DefaultCommand import DefaultCommand

class DefaultSwerveDrive(DefaultCommand):
    def __init__(
        self,
        mySwerveDrive: SwerveDrive,
        gameState:Callable[[],ShredderState] = lambda: None,
        frcFwd: Callable[[], float] = lambda: 0.0,
        frcLeft: Callable[[], float] = lambda: 0.0,
        frcRotation: Callable[[], float] = lambda: 0.0
    ) -> None:
        super().__init__(
            {
                ShredderState.DoNothing: DriveByStick( mySwerveDrive ).withName("GoNowhere"),
            },
            gameState,
            DriveByStick( mySwerveDrive, frcFwd, frcLeft, frcRotation )
        )