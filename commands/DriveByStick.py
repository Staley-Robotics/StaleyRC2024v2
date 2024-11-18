import typing

from wpimath import applyDeadband

from commands2 import Command, Subsystem
from subsystems.SwerveDrive import SwerveDrive

class DriveByStick(Command):
    # Deadband
    kDeadband = 0.04

    # Variable Declaration
    m_subsystem:SwerveDrive = None
    m_getX:typing.Callable[[],float] = lambda: 0.0
    m_getY:typing.Callable[[],float] = lambda: 0.0
    m_getRotation:typing.Callable[[],float] = lambda: 0.0
    
    # Initialization
    def __init__( self,
                  mySubsystem:SwerveDrive,
                  myX: typing.Callable[[], float] = lambda: 0.0,
                  myY: typing.Callable[[], float] = lambda: 0.0,
                  myRotation: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.m_subsystem:SwerveDrive = mySubsystem
        self.m_getX = myX
        self.m_getY = myY
        self.m_getRotation = myRotation
        self.setName( "DriveByStick" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.m_subsystem.runPercentInputs(
            applyDeadband( -self.m_getX(), self.kDeadband ),
            applyDeadband( -self.m_getY(), self.kDeadband ),
            applyDeadband( -self.m_getRotation(), self.kDeadband )
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