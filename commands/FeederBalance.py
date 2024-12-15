from commands2 import Command

from subsystems import Feeder, FeederModes
from util import *

class FeederBalance(Command):
    def __init__(self, feeder:Feeder):
        self.__feeder:Feeder = feeder

        self.setName( "FeederBalance" )
        self.addRequirements( feeder )

    def execute(self):
        if self.__feeder.topHasNote():
            self.__feeder.setSetpoint( FeederModes.BALANCEOUT )
        elif self.__feeder.bottomHasNote():
            self.__feeder.setSetpoint( FeederModes.BALANCEIN )
        else:
            self.__feeder.setSetpoint( FeederModes.STOP )
            #self.cancel()

    def end(self, interrupted):
        Crescendo.setState( ShredderState.HOLD_FEEDER if self.isFinished() else ShredderState.DEFAULT )
        self.__feeder.stop()
    
    def isFinished(self):
        return self.__feeder.bottomHasNote() == self.__feeder.topHasNote()
