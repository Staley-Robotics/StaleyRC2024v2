from pathlib import Path

from commands2 import Command, CommandScheduler

from wpilib import TimedRobot, DriverStation, DataLogManager, RobotBase

from RobotContainer import RobotContainer
from util import FalconLogger

class MyRobot(TimedRobot):
    # Variable Declaration
    __robotContainer:RobotContainer = None
    __autoCmd:Command = None
    __logger:FalconLogger = None

    # Initialization
    def robotInit(self):
        # Disable Notifications
        DriverStation.silenceJoystickConnectionWarning(True)

        # Start Logging using the built in DataLogManager
        logDir = '/U/logs' if RobotBase.isReal() else '.logs'
        DataLogManager.start( dir=(logDir if Path(logDir).is_dir() else ''), period=1.0 )
        DriverStation.startDataLog( DataLogManager.getLog() )
        
        # Built The Robot
        self.__robotContainer = RobotContainer()
        self.__logger = FalconLogger(False)

    # Periodic Loop / All Modes
    def robotPeriodic(self):
        self.__logger.setTime()
        CommandScheduler.getInstance().run()
        self.__logger.writeLog()

    # Autonomous Mode
    def autonomousInit(self):
        try:
            self.__autoCmd = self.__robotContainer.getAutonomousCommand()
            if self.__autoCmd != None:
                self.__autoCmd.schedule()
        except:
            print("WARNING! getAutonomousCommand failed!")
    
    def autonomousPeriodic(self): pass

    def autonomousExit(self):
        if self.__autoCmd != None:
            self.__autoCmd.cancel()

    # Teleop Mode
    def teleopInit(self): pass
    def teleopPeriodic(self): pass
    def teleopExit(self): pass

    # Test Mode
    def testInit(self): pass
    def testPeriodic(self): pass
    def testExit(self): pass

    # Disable Mode
    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

    # Simulation Mode
    def _simulationInit(self): pass
    def _simulationPeriodic(self): pass
    def _simulationExit(self): pass