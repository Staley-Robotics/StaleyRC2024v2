from commands2 import Command
from wpilib import Timer

from subsystems import Intake, IntakeOptions
from util.SharedConstants import SharedConstants

class IntakeEject(Command):  
    # Initialization
    def __init__( self,
                  mySubsystem:Intake
                ) -> None:
        # Command Attributes
        self.__intake:Intake = mySubsystem
        self.__timer:Timer = Timer()

        self.setName( "IntakeEject" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        self.__intake.setBrake( False )
        self.__timer.reset()
        self.__timer.start()

    # Periodic
    def execute(self) -> None:
        self.__intake.setSetpoint( IntakeOptions.EJECT )

    # On End
    def end(self, interrupted:bool) -> None:
        self.__intake.setSetpoint( IntakeOptions.STOP )

    # Is Finished
    def isFinished(self) -> bool:
        return self.__timer.hasElapsed( SharedConstants.EJECTTIME )

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False