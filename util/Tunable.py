# from wpilib import SmartDashboard

from ntcore import NetworkTable, NetworkTableInstance, EventFlags, Event

from typing import Any, Callable

from threading import Lock

class Tunable:

    nt_table: NetworkTable = None

    # GET_FUNCTIONS:dict[type, Callable] = {
    #     int:nt_instance.getNumber,
    #     float:nt_instance.getNumber,
    #     str:nt_instance.getString,
    #     bool:nt_instance.getBoolean
    # }
    # PUT_FUNCTIONS:dict[type, Callable] = {
    #     int:nt_instance.putNumber,
    #     float:nt_instance.putNumber,
    #     str:nt_instance.putString,
    #     bool:nt_instance.putBoolean
    # }
    # GET_TOPIC_FUNCS:dict[type, Callable] = {
    #     int:nt_table.getIntegerTopic,
    #     float:nt_table.getDoubleTopic,
    #     str:nt_table.getStringTopic,
    #     bool:nt_table.getBooleanTopic
    # }

    name:str = ''
    value:Any = None

    def __init__(self, path:str, name:str, value:Any, update_func:Callable):
        self.path = path
        self.name = name
        self.value = value
        self.type = type(value)
        self.update_func:Callable = update_func

        # self.lock = Lock()

        self.nt_table = NetworkTableInstance.getDefault().getTable(path)
        self.nt_entry = self.nt_table.getEntry(name)

        self.nt_table.addListener(name, EventFlags.kValueAll, self.update)

    def update(self, event:Event):
        # with self.lock:
        self.value = event.data.value.value()
        self.update_func()

    def get(self) -> Any:
        return self.value