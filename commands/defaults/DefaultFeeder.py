# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import Feeder

# Team Utility Imports
from util import DefaultCommand, ShredderState, Crescendo

class DefaultFeeder(DefaultCommand):
    def __init__(
        self,
        feederSubsystem:Feeder
    ):
        super().__init__(
            {
                ShredderState.HANDOFF: FeederHandoff( feederSubsystem ),
                ShredderState.UNBALANCED: FeederBalance( feederSubsystem ),
                ShredderState.SHOOTING: FeederLaunch( feederSubsystem )
            },
            Crescendo.getState
        )