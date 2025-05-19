from commands2 import Subsystem

from wpilib import RobotState, DigitalInput, SmartDashboard
from wpimath.controller import PIDController

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from dataclasses import dataclass

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration

@dataclass
class LauncherSpeeds:
    l_motor_mult:float = 1.0
    r_motor_mult:float = 0.8
    stop = 0
    max = 1
    intake = 0.8
    handoff = 0.4

class Launcher(Subsystem):

    m_logging:NetworkTable = None

    def __init__(self, left_motor_port:int, right_motor_port:int, ir_sensor_port:int):
        ## Subsystem Inits
        self.setName('Launcher')

        ## Logging Inits
        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/Launcher")

        ## Hardware Inits
        # motors
        self.left_motor = TalonFX( left_motor_port, 'canivore1' )
        self.right_motor = TalonFX( right_motor_port, 'canivore1' )
        motor_config = TalonFXConfiguration()
        self.left_motor.configurator.apply( motor_config )
        # motor_config.invert?
        self.right_motor.configurator.apply( motor_config )

        # ir sensor
        self.sensor = DigitalInput( ir_sensor_port )

        ## Function Inits
        self.speed_controller = PIDController(0.0,0.0,0.0)
        SmartDashboard.putData('Launcher/PID Controller', self.speed_controller)

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
        vels = self.getVelocities()
        self.m_logging.putNumber( "Measured Left", vels[0] )
        self.m_logging.putNumber( "Measured Right", vels[1] )

    def run(self) -> None:
        set_speed = self.speed_controller.calculate(self.left_motor.get_velocity())
        self.left_motor.set( set_speed * LauncherSpeeds.l_motor_mult )
        self.right_motor.set( set_speed * LauncherSpeeds.r_motor_mult )

    def stop(self) -> None:
        self.left_motor.set(LauncherSpeeds.stop)
        self.right_motor.set(LauncherSpeeds.stop)

    def setSpeed(self, value:float) -> None:
        self.speed_controller.setSetpoint( value )

    def getSetSpeed(self) -> float:
        return self.speed_controller.getSetpoint()

    def getVelocities(self) -> tuple[float, float]:
        return self.left_motor.get_velocity(), self.right_motor.get_velocity()
    
    def getSensorBeamBroken(self) -> True:
        return self.sensor.get()
