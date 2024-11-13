from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

from commands.SampleCommand import SampleCommand
from subsystems.Feeder import Feeder
from commands.FeederEject import FeederEject
from commands.FeederHandoff import FeederHandoff
from commands.FeederLaunch import FeederLaunch

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        # Declare Subsystems
        self.__feeder = Feeder()

        # Commands
        self.feederReceive = FeederHandoff(self.__feeder )
        self.feederLaunch = FeederLaunch(self.__feeder )
        self.feederEject = FeederEject(self.__feeder)

        # Autonomous Chooser
        self.m_autoChooser = SendableChooser()
        self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        # Default Commands
        #self.m_subsys.setDefaultCommand( self.leftX )

        # Driver Controller Button Binding
        #self.m_driver1.a().whileTrue( self.rightX )

        self.addDashboards( "Feeder", [self.feederReceive, self.feederLaunch, self.feederEject] )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        chooserValue = self.m_autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()
        
    def addDashboards( self, tabName:str, myCommands:list[Command] ):
        tab = Shuffleboard.getTab( tabName )
        for i in range(len(myCommands)):
            tab.add( myCommands[i].getName(), myCommands[i] )