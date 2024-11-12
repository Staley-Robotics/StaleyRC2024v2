from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard

#from commands.SampleCommand import SampleCommand
#from subsystems.SampleSubsystem import SampleSubsystem
from subsystems.Pivot import Pivot, PivotConstants
from commands.PivotToPosition import PivotToPosition

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        # Declare Subsystems
        #self.m_subsys = SampleSubsystem( 0 )
        self.pivot = Pivot()

        # Commands
        #self.leftX = SampleCommand(self.m_subsys, self.m_driver1.getLeftX )
        #self.rightX = SampleCommand(self.m_subsys, self.m_driver1.getRightX )
        self.pivotHigh    = PivotToPosition( self.pivot, PivotConstants.MAX )
        self.pivotAmp     = PivotToPosition( self.pivot, PivotConstants.AMP )
        self.pivotSpeaker = PivotToPosition( self.pivot, PivotConstants.SPEAKER )
        self.pivotHandoff = PivotToPosition( self.pivot, PivotConstants.HANDOFF )
        self.pivotToss    = PivotToPosition( self.pivot, PivotConstants.TOSS )
        self.pivotFlat    = PivotToPosition( self.pivot, PivotConstants.FLAT )
        self.pivotLow     = PivotToPosition( self.pivot, PivotConstants.MIN )

        # Autonomous Chooser
        self.m_autoChooser = SendableChooser()
        self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        # Default Commands
        #self.m_subsys.setDefaultCommand( self.leftX )

        # Driver Controller Button Binding
        #self.m_driver1.a().whileTrue( self.rightX )
        self.m_driver1.leftBumper().onTrue( self.pivotHigh )
        self.m_driver1.y().onTrue( self.pivotAmp )
        self.m_driver1.x().onTrue( self.pivotSpeaker )
        self.m_driver1.b().onTrue( self.pivotToss )
        self.m_driver1.a().onTrue( self.pivotFlat )
        self.m_driver1.rightBumper().onTrue( self.pivotLow )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        chooserValue = self.m_autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()