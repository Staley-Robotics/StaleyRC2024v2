from commands2 import Subsystem

from wpilib import RobotState, DigitalInput
from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from dataclasses import dataclass

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration

@dataclass
class IntakeSpeeds:
    stop = 0
    max = 1
    intake = 0.8
    handoff = 0.4

class Intake(Subsystem):

    m_logging:NetworkTable = None

    def __init__(self, upper_motor_port:int, lower_motor_port:int, ir_sensor_port:int):
        ## Subsystem Inits
        self.setName('Intake')

        ## Logging Inits
        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/Launcher")

        ## Hardware Inits
        # motors
        self.upper_motor = TalonFX( upper_motor_port, 'canivore1' )
        self.lower_motor = TalonFX( lower_motor_port, 'canivore1' )
        motor_config = TalonFXConfiguration()
        self.upper_motor.configurator.apply( motor_config )
        # motor_config.invert?
        self.lower_motor.configurator.apply( motor_config )

        # ir sensor
        self.sensor = DigitalInput( ir_sensor_port )

        ## Function Inits
        self.set_speed = ntproperty('Intake/Set Speed', 0, persistent=False, writeDefault=True)

    def periodic(self) -> None:
        ## Logging - Subsystem State
        # self.m_logging.putNumber( "SubsystemData", 0.0 )

        ## Run Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        ## Logging - Post Operation
        self.m_logging.putNumber( "Setpoint", self.getSetpoint() )
        # self.m_logging.putNumber( "Measured", self.m_system )

    def run(self) -> None:
        self.upper_motor.set(self.set_speed)
        self.upper_motor.set(self.set_speed)

    def stop(self) -> None:
        self.upper_motor.set(IntakeSpeeds.stop)
        self.upper_motor.set(IntakeSpeeds.stop)

    def setSpeed(self, value:float) -> None:
        self.set_speed = value

    def getSetSpeed(self) -> float:
        return self.set_speed

    def getSensorBeamBroken(self) -> True:
        return self.sensor.get()
