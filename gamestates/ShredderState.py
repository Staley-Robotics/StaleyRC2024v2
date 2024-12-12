from enum import Enum, auto

class ShredderState(Enum):
    DoNothing = auto()
    Pickup = auto()
    NoState = auto()
