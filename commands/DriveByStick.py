from typing import Callable

from wpimath import applyDeadband

from commands2 import Command, Subsystem

from subsystems.SwerveDrive import SwerveDrive

class DriveByStick(Command):
    # Variable Declaration
    m_subsystem:SwerveDrive = None
    vX:Callable[[],float] = lambda: 0.0
    vY:Callable[[],float] = lambda: 0.0
    rO:Callable[[],float] = lambda: 0.0

    controller_deadband = 0.04
    
    # Initialization
    def __init__( self,
                 swerveDrive:SwerveDrive,
                 velocityX:Callable,
                 velocityY:Callable,
                 rotation:Callable
                 ) -> None:
        # Command Setup
        self.drive = swerveDrive
        self.setName( "DriveByStick" )
        self.addRequirements( swerveDrive )

        # Drive Setup
        self.vX = velocityX
        self.vY = velocityY
        self.rO = rotation

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        xSpeed = applyDeadband(self.vX(), self.controller_deadband) * self.drive.k_maxSpeed
        ySpeed = applyDeadband(self.vY(), self.controller_deadband) * self.drive.k_maxSpeed
        rotSpeed = applyDeadband(self.rO(), self.controller_deadband) * self.drive.k_maxSpeed

        self.drive.drive(xSpeed, ySpeed, rotSpeed, True)

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False