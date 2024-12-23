# Python Imports
from typing import Callable

# WPI Imports
from commands2 import cmd

# Team Command Based Imports
from commands import *
from subsystems import SwerveDrive

# Team Utility Imports
from util import DefaultCommand
from util.Crescendo import *

class DefaultSwerveDrive(DefaultCommand):
    def __init__(
        self,
        driveSubsystem: SwerveDrive,
        frcFwd: Callable[[], float] = lambda: 0.0,
        frcLeft: Callable[[], float] = lambda: 0.0,
        frcRotation: Callable[[], float] = lambda: 0.0
    ) -> None:
        super().__init__(
            {
                ShredderState.NONE: DriveByStick( driveSubsystem ).withName("DoNothing"),
                # ShredderState.DEFAULT: DriveByStick( driveSubsystem ).withName("DEFAULT"),
                # ShredderState.PICKUP: DriveByStick( driveSubsystem ).withName("PICKUP"),
                # ShredderState.HOLD_INTAKE: DriveByStick( driveSubsystem ).withName("HOLD_INTAKE"),
                # ShredderState.HANDOFF: DriveByStick( driveSubsystem ).withName("HANDOFF"),
                # ShredderState.UNBALANCED: DriveByStick( driveSubsystem ).withName("UNBALANCED"),
                # ShredderState.HOLD_FEEDER: DriveByStick( driveSubsystem ).withName("HOLD_FEEDER"),
                # ShredderState.PREPARE_TO_SHOOT: DriveByStick( driveSubsystem ).withName("PREPARE_TO_SHOOT"),
                # ShredderState.READY_TO_SHOOT: DriveByStick( driveSubsystem ).withName("READY_TO_SHOOT"),
                # ShredderState.SHOOTING: DriveByStick( driveSubsystem ).withName("SHOOTING"),
                # ShredderState.SHOT_COMPLETE: DriveByStick( driveSubsystem ).withName("SHOT_COMPLETE")
            },
            Crescendo.getState,
            DriveByStick( driveSubsystem, frcFwd, frcLeft, frcRotation )
        )