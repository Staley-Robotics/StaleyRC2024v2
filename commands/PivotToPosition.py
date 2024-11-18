from commands2 import Command

from subsystems import Pivot

class PivotToPosition(Command):
    myPivot:Pivot = None

    def __init__(self, pivotSubsystem:Pivot, pivotPosition:float):
        self.__pivot:Pivot = pivotSubsystem
        self.__position:float = pivotPosition

        self.setName( f"PivotToPosition({self.__position})" )
        self.addRequirements( self.__pivot )

    def initialize(self):
        return self.__pivot.setSetpoint( self.__position )
    
    # def execute(self):
    #     print( f"{self.myPivot.atSetpoint()} {self.myPivot.getSetpoint()} {self.myPivot.getMeasurement()} {self.myPivot.m_motor.get()}")
    #     return super().execute()
    
    #def end(self, interrupted):
    #    return super().end(interrupted)
    
    def isFinished(self):
        return self.__pivot.atSetpoint()
    
    def runsWhenDisabled(self):
        return False