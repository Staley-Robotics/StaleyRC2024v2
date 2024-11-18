import typing

from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

from commands.SampleCommand import SampleCommand
from commands.DriveByStick import DriveByStick
from commands.FeederEject import FeederEject
from commands.FeederHandoff import FeederHandoff
from commands.FeederLaunch import FeederLaunch
from commands.PivotToPosition import PivotToPosition
from commands.LauncherStart import LauncherStart

from subsystems.SampleSubsystem import SampleSubsystem
from subsystems.SwerveDrive import SwerveDrive
from subsystems.Feeder import Feeder
from subsystems.Pivot import Pivot, PivotPositions
from subsystems.Launcher import Launcher, LauncherOptions

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        # Declare Subsystems
        self.m_driveTrain = SwerveDrive()
        self.m_intake = Intake()
        self.__feeder = Feeder()
        self.pivot = Pivot()
        self.__launcher = Launcher()

        # Commands
        self.driveCommand = DriveByStick(self.m_driveTrain, self.m_driver1.getLeftX, self.m_driver1.getLeftY, self.m_driver1.getRightY )
        self.intakeHandoff = IntakeHandoff( self.m_intake )
        self.intakePickup = IntakePickup( self.m_intake )
        self.intakeEject = IntakeEject( self.m_intake )
        self.feederReceive = FeederHandoff(self.__feeder )
        self.feederLaunch = FeederLaunch(self.__feeder )
        self.feederEject = FeederEject(self.__feeder)
        self.pivotHigh    = PivotToPosition( self.pivot, PivotPositions.MAX )
        self.pivotAmp     = PivotToPosition( self.pivot, PivotPositions.AMP )
        self.pivotSpeaker = PivotToPosition( self.pivot, PivotPositions.SPEAKER )
        self.pivotHandoff = PivotToPosition( self.pivot, PivotPositions.HANDOFF )
        self.pivotToss    = PivotToPosition( self.pivot, PivotPositions.TOSS )
        self.pivotFlat    = PivotToPosition( self.pivot, PivotPositions.FLAT )
        self.pivotLow     = PivotToPosition( self.pivot, PivotPositions.MIN )
        self.launchLong = LauncherStart( self.__launcher, LauncherOptions.LONG )
        self.launchToss = LauncherStart( self.__launcher, LauncherOptions.TOSS )
        self.launchAmp = LauncherStart( self.__launcher, LauncherOptions.AMP )
        self.launchStop = LauncherStart( self.__launcher, LauncherOptions.STOP )

        # Autonomous Chooser
        self.m_autoChooser = SendableChooser()
        self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        # Default Commands
        self.m_driveTrain.setDefaultCommand( self.driveCommand )

        # Driver Controller Button Binding
        self.m_driver1.a().toggleOnTrue( self.intakePickup )
        self.m_driver1.b().toggleOnTrue( self.intakeHandoff )
        
        # Dashboard Commands
        self.addDashboardCommands( "Intake", [self.intakeHandoff, self.intakePickup, self.intakeEject] )
        self.addDashboardCommands( "Feeder", [self.feederReceive, self.feederLaunch, self.feederEject] )
        self.addDashboardCommands( "Pivot", [self.pivotHigh, self.pivotAmp, self.pivotSpeaker, self.pivotHandoff, self.pivotToss, self.pivotFlat, self.pivotLow] )
        self.addDashboardCommands( "Launcher", [self.launchLong, self.launchToss, self.launchAmp, self.launchStop] )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        chooserValue = self.m_autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()
        
    # Publish Commands To Dashboards
    def addDashboardCommands( self, tabName:str, dashboardCommands:list[Command] ):
        tab = Shuffleboard.getTab( tabName )
        for i in range(len(dashboardCommands)):
            tab.add( dashboardCommands[i].getName(), dashboardCommands[i] )
