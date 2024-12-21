from enum import Enum, auto
from typing import Callable

from commands2 import Subsystem
from ntcore import NetworkTableInstance, NetworkTable

from subsystems.Pivot import PivotPositions

class DriveMode(Enum):
    NONE = auto()
    MANUAL = auto()
    AUTOMATIC = auto()
    CARDINAL = auto()

class ShredderMode(Enum):
    NONE = auto()
    TEST = auto()
    DEMO = auto()
    COMPETITION = auto()

class ShredderRegion(Enum):
    NONE = auto()
    FAR = auto()
    MIDDLE = auto()
    CLOSE = auto()

class ShredderState(Enum):
    NONE = auto()
    DEFAULT = auto()
    PICKUP = auto()
    HOLD_INTAKE = auto()
    WAIT_FOR_PIVOT = auto()
    HANDOFF = auto()
    UNBALANCED = auto()
    HOLD_FEEDER = auto()
    PREPARE_TO_SHOOT = auto()
    READY_TO_SHOOT = auto()
    SHOOTING = auto()
    SHOT_COMPLETE = auto()

class ShredderTarget(Enum):
    NONE = auto()
    SPEAKER = auto()
    AMP = auto()
    TOSS = auto()
    DEFENSE = auto()

class Crescendo(Subsystem):
    _currentMode:ShredderMode = ShredderMode.NONE
    _currentRegion:ShredderRegion = ShredderRegion.NONE
    _currentState:ShredderState = ShredderState.NONE
    _currentTarget:ShredderTarget = ShredderTarget.NONE

    __log:NetworkTable = NetworkTableInstance.getDefault().getTable( "/Shredder" )

    def __init__(self):
        self.setIntakeHasNote( lambda: False )
        self.setFeederHasBottomNote( lambda: False )
        self.setFeederHasTopNote( lambda: False )
        self.setPivotAtPosition( lambda: False )

        self.__log.putString( "Mode", str( self._currentMode ) )
        self.__log.putString( "Region", str( self._currentRegion ) )
        self.__log.putString( "State", str( self._currentState ) )
        self.__log.putString( "Target", str( self._currentTarget ) )

    def periodic(self):
        match self.getState():
            case ShredderState.DEFAULT:
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderState.UNBALANCED )
                elif self.__getFeederHasTopNote():
                    self.setState( ShredderState.HOLD_FEEDER )
                elif self.__getIntakeHasNote():
                    self.setState( ShredderState.HOLD_INTAKE ) 
            case ShredderState.PICKUP:
                pass
            case ShredderState.HOLD_INTAKE:
                if not self.__getIntakeHasNote():
                    self.setState( ShredderState.DEFAULT )
                elif self.__getPivotAtPosition( PivotPositions.HANDOFF ):
                    self.setState( ShredderState.HANDOFF )
            case ShredderState.WAIT_FOR_PIVOT:
                pass
                # if self.__getPivotAtPosition( PivotPositions.HANDOFF ):
                #     self.setState( ShredderState.HOLD_INTAKE )
            case ShredderState.HANDOFF:
                if not( self.__getIntakeHasNote() or self.__getFeederHasTopNote() or self.__getFeederHasBottomNote() ):
                    self.setState( ShredderState.DEFAULT )
            case ShredderState.HOLD_FEEDER:
                if not( self.__getFeederHasBottomNote() or self.__getFeederHasTopNote() ):
                    self.setState( ShredderState.DEFAULT )
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderState.UNBALANCED )
            case ShredderState.PREPARE_TO_SHOOT:
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderState.UNBALANCED )
            case ShredderState.READY_TO_SHOOT:
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderState.UNBALANCED )

    @classmethod
    def getState(self) -> ShredderState:
        return self._currentState
    
    @classmethod
    def setState(self, state:ShredderState) -> None:
        self.__log.putString( "State", str( state ) )
        self._currentState = state

    #@classmethod
    def setNextState(self) -> None:
        match self.getState():
            case ShredderState.DEFAULT:
                self.setState( ShredderState.PICKUP )
            case ShredderState.PICKUP:
                self.setState( ShredderState.DEFAULT )
            case ShredderState.HOLD_INTAKE:
                if self.__getPivotAtPosition( position=float(PivotPositions.HANDOFF) ):
                    self.setState( ShredderState.HANDOFF )
                else:
                    self.setState( ShredderState.WAIT_FOR_PIVOT )
            case ShredderState.WAIT_FOR_PIVOT:
                if self.__getPivotAtPosition( position=float(PivotPositions.HANDOFF) ):
                    self.setState( ShredderState.HANDOFF )
            case ShredderState.HOLD_FEEDER:
                self.setState( ShredderState.PREPARE_TO_SHOOT )
            case ShredderState.READY_TO_SHOOT:
                self.setState( ShredderState.SHOOTING )

    @classmethod
    def getTarget(self) -> ShredderTarget:
        return self._currentTarget

    @classmethod
    def setTarget(self, target:ShredderTarget) -> None:
        self._currentTarget = target
        self.__log.putString( "Target", str( target ) )

    @classmethod
    def setNextTarget(self) -> None:
        match self.getTarget():
            case ShredderTarget.NONE:
                self.setTarget( ShredderTarget.SPEAKER )
            case ShredderTarget.AMP:
                self.setTarget( ShredderTarget.TOSS )
            case ShredderTarget.SPEAKER:
                self.setTarget( ShredderTarget.AMP )
            case ShredderTarget.TOSS:
                self.setTarget( ShredderTarget.SPEAKER )

        ## Change Shredder State based on ShreddedState here
        match self.getState():
            case ShredderState.PREPARE_TO_SHOOT:
                self.setState( ShredderState.HOLD_FEEDER )
            case ShredderState.READY_TO_SHOOT:
                self.setState( ShredderState.HOLD_FEEDER )
    

    @classmethod
    def getMode(self) -> ShredderMode:
        return self._currentMode

    def setMode(self, mode:ShredderMode) -> None:
        self._currentMode = mode
        self.__log.putString( "Mode", str( mode ) )

    @classmethod
    def getPosition(self) -> ShredderRegion:
        return self._currentRegion

    @classmethod
    def getDriveMode() -> DriveMode:
        return 
    
    def setIntakeHasNote( self, func:Callable[[],bool] ):
        self.__lambdaIntakeHasNote = func

    def setFeederHasBottomNote( self, func:Callable[[],bool] ):
        self.__lambdaFeederHasBottomNote = func

    def setFeederHasTopNote( self, func:Callable[[],bool] ):
        self.__lambdaFeederHasTopNote = func

    def setPivotAtPosition( self, func:Callable[[float],bool] ):
        self.__lambdaPivotAtPosition = func

    ### LAMBDAS????
    def __getIntakeHasNote(self) -> bool:
        return self.__lambdaIntakeHasNote()
    
    def __getFeederHasTopNote(self) -> bool:
        return self.__lambdaFeederHasTopNote()
    
    def __getFeederHasBottomNote(self) -> bool:
        return self.__lambdaFeederHasBottomNote()
    
    def __getPivotAtPosition(self, position:float) -> bool:
        return self.__lambdaPivotAtPosition( position )
