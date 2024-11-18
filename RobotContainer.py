from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd
from wpilib import SendableChooser, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

from commands.LauncherStart import LauncherStart
from subsystems.Launcher import Launcher, LauncherOptions

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    # Initialization
    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        # Declare Subsystems
        self.__launcher = Launcher()

        # Commands
        self.launchLong = LauncherStart( self.__launcher, LauncherOptions.LONG )
        self.launchToss = LauncherStart( self.__launcher, LauncherOptions.TOSS )
        self.launchAmp = LauncherStart( self.__launcher, LauncherOptions.AMP )
        self.launchStop = LauncherStart( self.__launcher, LauncherOptions.STOP )

        # Autonomous Chooser
        self.m_autoChooser = SendableChooser()
        self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        # Default Commands
        #self.m_subsys.setDefaultCommand( self.leftX )

        # Driver Controller Button Binding
        self.m_driver1.a().toggleOnTrue( self.launchLong )
        self.m_driver1.b().toggleOnTrue( self.launchToss )
        self.m_driver1.x().toggleOnTrue( self.launchAmp )

        # Dashboarding
        self.addDashboards( "Launcher", [ self.launchLong, self.launchToss, self.launchAmp, self.launchStop ])

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