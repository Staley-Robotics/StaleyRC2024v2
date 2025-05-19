from enum import Enum, auto
from typing import Callable
from commands2 import Subsystem
from ntcore import NetworkTableInstance, NetworkTable
from subsystems import PivotPositions

class DriveModes(Enum):
    NONE = auto()
    MANUAL = auto()
    AUTOMATIC = auto()
    CARDINAL = auto()

class ShredderModes(Enum):
    NONE = auto()
    TEST = auto()
    DEMO = auto()
    COMPETITION = auto()

class TargetDistances(Enum):
    NONE = auto()
    FAR = auto()
    MIDDLE = auto()
    CLOSE = auto()

class ShredderStates(Enum):
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

class GameTargets(Enum):
    NONE = auto()
    SPEAKER = auto()
    AMP = auto()
    TOSS = auto()
    DEFENSE = auto()

class RobotState(Subsystem):
    _currentMode:ShredderModes = ShredderModes.NONE
    _currentRegion:TargetDistances = TargetDistances.NONE
    _currentState:ShredderStates = ShredderStates.NONE
    _currentTarget:GameTargets = GameTargets.NONE

    __log:NetworkTable = NetworkTableInstance.getDefault().getTable( "/Shredder" )

    def __init__(self):
        self.setIntakeHasNote( lambda: False )
        self.setFeederHasBottomNote( lambda: False )
        self.setFeederHasTopNote( lambda: False )
        self.setPivotAtPosition( lambda: False )

        self.__log.putString( "Mode", str( self._currentMode ) )
        self.__log.putString( "Distances", str( self._currentRegion ) )
        self.__log.putString( "State", str( self._currentState ) )
        self.__log.putString( "Target", str( self._currentTarget ) )

    def periodic(self):
        match self.getState():
            case ShredderStates.DEFAULT:
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderStates.UNBALANCED )
                elif self.__getFeederHasTopNote():
                    self.setState( ShredderStates.HOLD_FEEDER )
                elif self.__getIntakeHasNote():
                    self.setState( ShredderStates.HOLD_INTAKE ) 
            case ShredderStates.PICKUP:
                pass
            case ShredderStates.HOLD_INTAKE:
                if not self.__getIntakeHasNote():
                    self.setState( ShredderStates.DEFAULT )
                elif self.__getPivotAtPosition( PivotPositions.HANDOFF ):
                    self.setState( ShredderStates.HANDOFF )
            case ShredderStates.WAIT_FOR_PIVOT:
                pass
                # if self.__getPivotAtPosition( PivotPositions.HANDOFF ):
                #     self.setState( ShredderState.HOLD_INTAKE )
            case ShredderStates.HANDOFF:
                if not( self.__getIntakeHasNote() or self.__getFeederHasTopNote() or self.__getFeederHasBottomNote() ):
                    self.setState( ShredderStates.DEFAULT )
            case ShredderStates.HOLD_FEEDER:
                if not( self.__getFeederHasBottomNote() or self.__getFeederHasTopNote() ):
                    self.setState( ShredderStates.DEFAULT )
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderStates.UNBALANCED )
            case ShredderStates.PREPARE_TO_SHOOT:
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderStates.UNBALANCED )
            case ShredderStates.READY_TO_SHOOT:
                if self.__getFeederHasTopNote() != self.__getFeederHasBottomNote():
                    self.setState( ShredderStates.UNBALANCED )

    @classmethod
    def getState(self) -> ShredderStates:
        return self._currentState
    
    @classmethod
    def setState(self, state:ShredderStates) -> None:
        self.__log.putString( "State", str( state ) )
        self._currentState = state

    #@classmethod
    def setNextState(self) -> None:
        match self.getState():
            case ShredderStates.DEFAULT:
                self.setState( ShredderStates.PICKUP )
            case ShredderStates.PICKUP:
                self.setState( ShredderStates.DEFAULT )
            case ShredderStates.HOLD_INTAKE:
                if self.__getPivotAtPosition( position=float(PivotPositions.HANDOFF) ):
                    self.setState( ShredderStates.HANDOFF )
                else:
                    self.setState( ShredderStates.WAIT_FOR_PIVOT )
            case ShredderStates.WAIT_FOR_PIVOT:
                if self.__getPivotAtPosition( position=float(PivotPositions.HANDOFF) ):
                    self.setState( ShredderStates.HANDOFF )
            case ShredderStates.HOLD_FEEDER:
                self.setState( ShredderStates.PREPARE_TO_SHOOT )
            case ShredderStates.READY_TO_SHOOT:
                self.setState( ShredderStates.SHOOTING )

    @classmethod
    def getTarget(self) -> GameTargets:
        return self._currentTarget

    @classmethod
    def setTarget(self, target:GameTargets) -> None:
        self._currentTarget = target
        self.__log.putString( "Target", str( target ) )

    @classmethod
    def setNextTarget(self) -> None:
        match self.getTarget():
            case GameTargets.NONE:
                self.setTarget( GameTargets.SPEAKER )
            case GameTargets.AMP:
                self.setTarget( GameTargets.TOSS )
            case GameTargets.SPEAKER:
                self.setTarget( GameTargets.AMP )
            case GameTargets.TOSS:
                self.setTarget( GameTargets.SPEAKER )

        ## Change Shredder State based on ShreddedState here
        match self.getState():
            case ShredderStates.PREPARE_TO_SHOOT:
                self.setState( ShredderStates.HOLD_FEEDER )
            case ShredderStates.READY_TO_SHOOT:
                self.setState( ShredderStates.HOLD_FEEDER )
    

    @classmethod
    def getMode(self) -> ShredderModes:
        return self._currentMode

    def setMode(self, mode:ShredderModes) -> None:
        self._currentMode = mode
        self.__log.putString( "Mode", str( mode ) )

    @classmethod
    def getPosition(self) -> TargetDistances:
        return self._currentRegion

    @classmethod
    def getDriveMode() -> DriveModes:
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