from commands2 import Command
from commands2.button import CommandXboxController, Trigger
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

from commands import *
from subsystems import *

class RobotContainer:
    # Variable Declaration
    __autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        driver1 = CommandXboxController( 0 )

        # Declare Subsystems
        sysDriveTrain = SwerveDrive()
        sysIntake = Intake()
        sysFeeder = Feeder()
        sysPivot = Pivot()
        sysLauncher = Launcher()

        # Commands
        cmdDriveCommand  = DriveByStick( sysDriveTrain, driver1.getLeftX, driver1.getLeftY, driver1.getRightY )
        cmdIntakeHandoff = IntakeHandoff( sysIntake )
        cmdIntakePickup  = IntakePickup( sysIntake )
        cmdIntakeEject   = IntakeEject( sysIntake )
        cmdFeederReceive = FeederHandoff( sysFeeder )
        cmdFeederLaunch  = FeederLaunch( sysFeeder )
        cmdFeederEject   = FeederEject( sysFeeder )
        cmdFeederBalance = FeederBalance( sysFeeder )
        cmdPivotHigh     = PivotToPosition( sysPivot, PivotPositions.MAX )
        cmdPivotAmp      = PivotToPosition( sysPivot, PivotPositions.AMP )
        cmdPivotSpeaker  = PivotToPosition( sysPivot, PivotPositions.SPEAKER )
        cmdPivotHandoff  = PivotToPosition( sysPivot, PivotPositions.HANDOFF )
        cmdPivotToss     = PivotToPosition( sysPivot, PivotPositions.TOSS )
        cmdPivotFlat     = PivotToPosition( sysPivot, PivotPositions.FLAT )
        cmdPivotLow      = PivotToPosition( sysPivot, PivotPositions.MIN )
        cmdLaunchLong    = LauncherStart( sysLauncher, LauncherOptions.LONG )
        cmdLaunchToss    = LauncherStart( sysLauncher, LauncherOptions.TOSS )
        cmdLaunchAmp     = LauncherStart( sysLauncher, LauncherOptions.AMP )
        cmdLaunchStop    = LauncherStop( sysLauncher )

        # Sequences
        seqPickup = cmdIntakePickup.alongWith( cmdPivotHandoff ).andThen( cmdFeederReceive.alongWith( cmdIntakeHandoff ) )
        seqPickup = seqPickup.withName( "PickupSequence" )
        seqLaunch = cmdLaunchLong.andThen( cmdFeederLaunch ).andThen( cmdLaunchStop )
        seqLaunch = seqLaunch.withName( "LaunchSequence" )

        # Special Command Handling
        sysFeeder.addBalanceCommand( cmdFeederBalance )

        # Autonomous Chooser
        self.__autoChooser = SendableChooser()
        self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
        sysDriveTrain.setDefaultCommand( cmdDriveCommand )

        # Driver Controller Button Binding
        driver1.a().toggleOnTrue( cmdIntakePickup )
        driver1.b().toggleOnTrue( cmdIntakeHandoff )
        driver1.x().and_( lambda: not sysFeeder.hasSecuredNote() ).toggleOnTrue( seqPickup )
        driver1.y().and_( lambda: sysFeeder.hasSecuredNote() ).toggleOnTrue( seqLaunch )
        
        # Dashboard Commands
        self.addDashboardCommands( "Intake",   [cmdIntakeHandoff, cmdIntakePickup, cmdIntakeEject] )
        self.addDashboardCommands( "Feeder",   [cmdFeederReceive, cmdFeederLaunch, cmdFeederEject, cmdFeederBalance] )
        self.addDashboardCommands( "Pivot",    [cmdPivotHigh, cmdPivotAmp, cmdPivotSpeaker, cmdPivotHandoff, cmdPivotToss, cmdPivotFlat, cmdPivotLow] )
        self.addDashboardCommands( "Launcher", [cmdLaunchLong, cmdLaunchToss, cmdLaunchAmp, cmdLaunchStop] )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        chooserValue = self.__autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()
        
    # Publish Commands To Dashboards
    def addDashboardCommands( self, tabName:str, dashboardCommands:list[Command] ):
        tab = Shuffleboard.getTab( tabName )
        for i in range(len(dashboardCommands)):
            tab.add( dashboardCommands[i].getName(), dashboardCommands[i] )
