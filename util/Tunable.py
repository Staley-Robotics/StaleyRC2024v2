# from wpilib import SmartDashboard

from ntcore import NetworkTable, NetworkTableInstance

from typing import Any, Callable

class Tunable:

    nt_instance: NetworkTable = None



    name:str = ''
    value:Any = None

    def __init__(self, path:str, name:str, value:Any):
        self.path = path
        self.name = name
        self.value = value
        self.type = type(value)

        self.nt_instance = NetworkTableInstance.getDefault().getTable(path)
        self.GET_FUNCTIONS:dict[type, Callable] = {
            int:self.nt_instance.getNumber,
            float:self.nt_instance.getNumber,
            str:self.nt_instance.getString,
            bool:self.nt_instance.getBoolean
        }
        self.PUT_FUNCTIONS:dict[type, Callable] = {
            int:self.nt_instance.putNumber,
            float:self.nt_instance.putNumber,
            str:self.nt_instance.putString,
            bool:self.nt_instance.putBoolean
        }
    
    def get(self) -> Any:
        return self.value
    
    def update(self) -> Any:
        self.value = self.GET_FUNCTIONS[self.type](self.name, self.value)
        return self.value
    
    def set(self, value:Any) -> None:
        self.value = value
        self.PUT_FUNCTIONS[self.type](self.name, self.value)