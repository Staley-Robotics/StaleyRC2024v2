# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import Intake

# Team Utility Imports
from util import DefaultCommand, ShredderState, Crescendo

class DefaultIntake(DefaultCommand):
    def __init__(
        self,
        intakeSubsystem:Intake
    ):
        super().__init__(
            {
                ShredderState.PICKUP: IntakePickup( intakeSubsystem ),
                ShredderState.HANDOFF: IntakeHandoff( intakeSubsystem )
            },
            Crescendo.getState
        )