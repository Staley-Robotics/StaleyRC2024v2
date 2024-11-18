import typing

from wpimath import applyDeadband
from commands2 import Command

from subsystems import SwerveDrive

class DriveByStick(Command):
    # Deadband
    kDeadband = 0.04

    # Initialization
    def __init__( self,
                  mySubsystem:SwerveDrive,
                  myX: typing.Callable[[], float] = lambda: 0.0,
                  myY: typing.Callable[[], float] = lambda: 0.0,
                  myRotation: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.__subsystem:SwerveDrive = mySubsystem
        self.__getX = myX
        self.__getY = myY
        self.__getRotation = myRotation
        
        self.setName( "DriveByStick" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.__subsystem.runPercentInputs(
            applyDeadband( -self.__getX(), self.kDeadband ),
            applyDeadband( -self.__getY(), self.kDeadband ),
            applyDeadband( -self.__getRotation(), self.kDeadband )
        )

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False