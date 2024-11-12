import typing

from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

from commands.IntakePickup import IntakePickup
from commands.IntakeHandoff import IntakeHandoff
from commands.IntakeEject import IntakeEject
from subsystems.Intake import Intake

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        # Declare Subsystems
        self.m_intake = Intake()

        # Commands
        #self.leftX = SampleCommand(self.m_subsys, self.m_driver1.getLeftX )
        #self.rightX = SampleCommand(self.m_subsys, self.m_driver1.getRightX )
        self.intakeHandoff = IntakeHandoff( self.m_intake )
        self.intakePickup = IntakePickup( self.m_intake )
        self.intakeEject = IntakeEject( self.m_intake )

        # Autonomous Chooser
        self.m_autoChooser = SendableChooser()
        self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        # Default Commands
        #self.m_subsys.setDefaultCommand( self.leftX )

        # Driver Controller Button Binding
        self.m_driver1.a().toggleOnTrue( self.intakePickup )
        self.m_driver1.b().toggleOnTrue( self.intakeHandoff )

        # Shuffleboard Buttons for Troubleshooting
        self.addDashboardCommands( "Intake", [self.intakeHandoff, self.intakePickup, self.intakeEject] )

    def addDashboardCommands( self, tabName:str, dashboardCommands:list[Command] ) -> None:
        tab = Shuffleboard.getTab( tabName )
        for x in range(len(dashboardCommands)):
            myCmd:Command = dashboardCommands[x]
            tab.add( title=f"{myCmd.getName()}", defaultValue=myCmd )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        chooserValue = self.m_autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()