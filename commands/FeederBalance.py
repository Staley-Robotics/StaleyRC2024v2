from commands2 import Command

from subsystems import Feeder, FeederModes

class FeederBalance(Command):
    def __init__(self, feeder:Feeder):
        self.setName( "FeederBalance" )
        self.addRequirements( feeder )
        self.__feeder:Feeder = feeder

    def initialize(self):
        if self.__feeder.topHasNote():
            self.__feeder.setSetpoint( FeederModes.BALANCEOUT )
        elif self.__feeder.bottomHasNote():
            self.__feeder.setSetpoint( FeederModes.BALANCEIN )
        else:
            self.__feeder.setSetpoint( FeederModes.STOP )

    def end(self, interrupted):
        self.__feeder.stop()
    
    def isFinished(self):
        return self.__feeder.bottomHasNote() == self.__feeder.topHasNote()
