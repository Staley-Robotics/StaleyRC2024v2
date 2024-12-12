import typing

from wpimath import applyDeadband
from commands2 import Command

from Constants import *
from subsystems import SwerveDrive

class DriveByStick(Command):
    # Deadband
    kDeadband = 0.04

    # Initialization
    # myX assumes forward is positive
    # myY assumes left is positive
    # myRotation assumes CCW is positive
    def __init__( self,
                  mySubsystem:SwerveDrive,
                  frcFwd: typing.Callable[[], float] = lambda: 0.0,
                  frcLeft: typing.Callable[[], float] = lambda: 0.0,
                  frcRotation: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.__subsystem:SwerveDrive = mySubsystem
        
        self.__getX = frcFwd
        self.__getY = frcLeft
        self.__getRotation = frcRotation
        
        self.setName( "DriveByStick" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.__subsystem.runPercentInputs( self.getX(), self.getY(), self.getR() )

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
    
    # calls __getX lambda with Slew Rate Limiter Integration
    def getX(self, close:bool = False) -> float:
        x = self.__getX()
        x_normal = Constants.Limiters.srl_tX.calculate( x )
        x_close = Constants.Limiters.srl_tX_close.calculate( x )
        return x_normal if not close else x_close
    
    # calls __getX lambda with Slew Rate Limiter Integration
    def getY(self, close:bool = False) -> float:
        y = self.__getY()
        y_normal = Constants.Limiters.srl_tY.calculate( y )
        y_close = Constants.Limiters.srl_tY_close.calculate( y )       
        return y_normal if not close else y_close
    
    # calls __getRotation lambda with Slew Rate Limiter Integration
    def getR(self, close:bool = False) -> float:
        r = self.__getRotation()
        r_normal = Constants.Limiters.srl_rO.calculate( r )
        r_close = Constants.Limiters.srl_rO_close.calculate( r )
        return r_normal if not close else r_close