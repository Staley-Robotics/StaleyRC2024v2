from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard

from commands import IndexerEJECT, IndexerHANDOFF, IndexerSTOP
from subsystems.Indexer import Indexer

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        # Declare Subsystems
        self.m_Indexer = Indexer( 0 ) #idk the device id yay

        # Commands
        self.indexer_stop = IndexerSTOP.IndexerSTOP(self.m_Indexer)
        self.indexer_eject = IndexerEJECT.IndexerEJECT(self.m_Indexer)
        self.indexer_handoff = IndexerHANDOFF.IndexerHANDOFF(self.m_Indexer)
        # self.leftX = SampleCommand(self.m_subsys, self.m_driver1.getLeftX )
        # self.rightX = SampleCommand(self.m_subsys, self.m_driver1.getRightX )

        # Autonomous Chooser
        self.m_autoChooser = SendableChooser()
        self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        # Default Commands
        # self.m_Indexer.setDefaultCommand( self.leftX )

        # Driver Controller Button Binding
        self.m_driver1.a().whileTrue( self.indexer_stop ) # A ==> stop
        self.m_driver1.x().whileTrue( self.indexer_eject ) # X ==> eject
        self.m_driver1.b().whileTrue( self.indexer_handoff ) # B ==> handoff

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        chooserValue = self.m_autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()