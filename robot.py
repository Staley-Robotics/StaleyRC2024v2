from pathlib import Path

from commands2 import Command, CommandScheduler

from wpilib import TimedRobot, DriverStation, DataLogManager, RobotBase

from RobotContainer import RobotContainer

class MyRobot(TimedRobot):
    # Variable Declaration
    __robotContainer:RobotContainer = None
    __autoCmd:Command = None

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

    # Periodic Loop / All Modes
    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    # Autonomous Mode
    def autonomousInit(self):
        self.__autoCmd = self.__robotContainer.getAutonomousCommand()
        if self.__autoCmd != None:
            self.__autoCmd.schedule()
    
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