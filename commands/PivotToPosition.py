from commands2 import Command
from wpilib import SmartDashboard

from subsystems.Pivot import Pivot

class PivotToPosition(Command):
    myPivot:Pivot = None

    def __init__(self, pivotSubsystem:Pivot, pivotPosition:float):
        self.myPivot = pivotSubsystem
        self.myPosition = pivotPosition

        self.setName( f"PivotToPosition({self.myPosition})" )
        self.addRequirements( self.myPivot )

        super().__init__()

    def initialize(self):
        return self.myPivot.setSetpoint( self.myPosition )
    
    # def execute(self):
    #     print( f"{self.myPivot.atSetpoint()} {self.myPivot.getSetpoint()} {self.myPivot.getMeasurement()} {self.myPivot.m_motor.get()}")
    #     return super().execute()
    
    #def end(self, interrupted):
    #    return super().end(interrupted)
    
    def isFinished(self):
        return self.myPivot.atSetpoint()
    
    def runsWhenDisabled(self):
        return False