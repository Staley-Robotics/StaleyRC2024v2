from commands2 import Command, InstantCommand
#from commands2.button import CommandXboxController, Trigger
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

# from pathplannerlib.auto import AutoBuilder, NamedCommands

from commands import *
from commands.defaults import *
from subsystems import *
from util import *

class RobotContainer:
    # Variable Declaration
    __autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        driver1 = FalconXboxController( 0 )

        # Declare Subsystems
        sysCrescendo = Crescendo()
        sysCrescendo.setState( ShredderState.DEFAULT )
        sysDriveTrain = SwerveDrive()
        sysIntake = Intake()
        sysFeeder = Feeder()
        sysPivot = Pivot()
        sysLauncher = Launcher()

        # Cameras
        sysLimelight1 = Vision( "limelight-one", sysDriveTrain.getOdometry )
        sysLimelight2 = Vision( "limelight-two", sysDriveTrain.getOdometry )
        sysLimelight3 = Vision( "limelight-three", sysDriveTrain.getOdometry )
        sysLimelight4 = Vision( "limelight-four", sysDriveTrain.getOdometry )

        # Commands
        cmdDriveCommand  = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )
        cmdIntakeHandoff = IntakeHandoff( sysIntake )
        cmdIntakePickup  = IntakePickup( sysIntake ) #.onlyIf( lambda: not sysFeeder.hasSecuredNote() ).withName( "IntakePickup" )
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

        # Default Commands
        defaultSwerveDrive = DefaultSwerveDrive( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )
        defaultIntake = DefaultIntake( sysIntake )
        defaultPivot = DefaultPivot( sysPivot )
        defaultFeeder = DefaultFeeder( sysFeeder )
        defaultLauncher = DefaultLauncher( sysLauncher )

        # Sequences
        seqPickupIntake = IntakePickup( sysIntake )
        seqPickupPivot = PivotToPosition( sysPivot, PivotPositions.HANDOFF )
        seqPickupFeeder = FeederHandoff( sysFeeder )
        seqPickupIntakeHandoff = IntakeHandoff( sysIntake )
        seqPickup = seqPickupIntake.andThen( seqPickupPivot ).andThen( seqPickupFeeder.alongWith( seqPickupIntakeHandoff ) )
        seqPickup = seqPickup.onlyIf( lambda: not sysFeeder.hasSecuredNote() )
        seqPickup = seqPickup.withName( "PickupSequence" )

        seqLaunchStart = LauncherStart( sysLauncher, LauncherOptions.LONG )
        seqLaunchFeeder = FeederLaunch( sysFeeder )
        seqLaunchStop = LauncherStop( sysLauncher )
        seqLaunch = seqLaunchStart.andThen( seqLaunchFeeder ).andThen( cmd.waitSeconds( 0.25 ) ).andThen( seqLaunchStop )
        seqLaunch = seqLaunch.onlyIf( lambda: sysFeeder.hasSecuredNote() )
        seqLaunch = seqLaunch.withName( "LaunchSequence" )

        # Reset Gyro Command
        cmdDriveResetGyro = cmd.runOnce( lambda: sysDriveTrain.resetGyro() ).ignoringDisable(True)
        
        # Vision Use Locked In Range Command
        cmdVTLIR1 = cmd.runOnce( lambda: sysLimelight1.toggleUseLockedInRange() )
        cmdVTLIR2 = cmd.runOnce( lambda: sysLimelight2.toggleUseLockedInRange() )
        cmdVTLIR3 = cmd.runOnce( lambda: sysLimelight3.toggleUseLockedInRange() )
        cmdVTLIR4 = cmd.runOnce( lambda: sysLimelight4.toggleUseLockedInRange() )
        cmdVisionTLIR = cmdVTLIR1.alongWith( cmdVTLIR2 ).alongWith( cmdVTLIR3 ).alongWith( cmdVTLIR4 ).ignoringDisable(True)

        # Special Command Handling
        sysFeeder.addBalanceCommand( cmdFeederBalance )

        # PathPlanner Register Named Commands
        # NamedCommands.registerCommand('Pickup', seqPickup )
        # NamedCommands.registerCommand('LaunchSpeaker', seqLaunch )

        # Autonomous Chooser
        # self.__autoChooser = AutoBuilder.buildAutoChooser( "None" )
        # SmartDashboard.putData( "Auto Chooser", self.__autoChooser )

        # Default Commands
        sysDriveTrain.setDefaultCommand( defaultSwerveDrive )
        sysIntake.setDefaultCommand( defaultIntake )
        sysPivot.setDefaultCommand( defaultPivot )
        sysFeeder.setDefaultCommand( defaultFeeder )
        sysLauncher.setDefaultCommand( defaultLauncher )

        # Driver Controller Button Binding
        # driver1.a().toggleOnTrue( cmdIntakePickup )
        # driver1.b().toggleOnTrue( cmdIntakeHandoff )
        # driver1.x().toggleOnTrue( seqPickup )
        # driver1.y().toggleOnTrue( seqLaunch )
        # driver1.start().onTrue( cmdDriveResetGyro )
        # driver1.back().onTrue( cmdVisionTLIR )

        sysCrescendo.setIntakeHasNote( sysIntake.hasNote )
        sysCrescendo.setFeederHasBottomNote( sysFeeder.bottomHasNote )
        sysCrescendo.setFeederHasTopNote( sysFeeder.topHasNote )
        sysCrescendo.setPivotAtPosition( sysPivot.atSetpoint )

        driver1.a().onTrue( InstantCommand( lambda: sysCrescendo.setNextState() ) )
        driver1.b().onTrue( InstantCommand( lambda: sysCrescendo.setNextTarget() ) )
        driver1.x().onTrue( InstantCommand( lambda: defaultLauncher.toggleAutoStart() ) )
        
        # Dashboard Commands
        self.addDashboardCommands( "Intake",   [cmdIntakeHandoff, cmdIntakePickup, cmdIntakeEject] )
        self.addDashboardCommands( "Feeder",   [cmdFeederReceive, cmdFeederLaunch, cmdFeederEject, cmdFeederBalance] )
        self.addDashboardCommands( "Pivot",    [cmdPivotHigh, cmdPivotAmp, cmdPivotSpeaker, cmdPivotHandoff, cmdPivotToss, cmdPivotFlat, cmdPivotLow] )
        self.addDashboardCommands( "Launcher", [cmdLaunchLong, cmdLaunchToss, cmdLaunchAmp, cmdLaunchStop] )
        self.addDashboardCommands( "Sequences", [seqPickup, seqLaunch] )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
        
    # Publish Commands To Dashboards
    def addDashboardCommands( self, tabName:str, dashboardCommands:list[Command] ):
        tab = Shuffleboard.getTab( tabName )
        for i in range(len(dashboardCommands)):
            name = dashboardCommands[i].getName() if dashboardCommands[i].getName() is not None else "CustomCommand"
            tab.add( dashboardCommands[i].getName(), dashboardCommands[i] )
