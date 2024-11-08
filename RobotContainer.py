from wpilib import SendableChooser, SmartDashboard

# from phoenix6.hardware import Pigeon2

from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd

from commands.DriveByStick import DriveByStick
from subsystems.SwerveDrive.SwerveDrive import SwerveDrive
from subsystems.SwerveDrive.SwerveModule import SwerveModule
from subsystems.SwerveDrive.W_Pigeon2 import W_Pigeon2

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        ## Declare Subsystems
        #Drive
        swerve_modules = [
            SwerveModule("FL", 7, 8, 18, 97.471 ),
            SwerveModule("FR", 1, 2, 12, 5.361 ),
            SwerveModule("BL", 5, 6, 16, 298.828 ),
            SwerveModule("BR", 3, 4, 14, 60.557 )
        ]
        gyro = W_Pigeon2( 9, 'rio')
        self.drive = SwerveDrive( 'rio', swerve_modules, gyro )

        ## Commands
        # self.leftX = SampleCommand(self.m_subsys, self.m_driver1.getLeftX )
        # self.rightX = SampleCommand(self.m_subsys, self.m_driver1.getRightX )

        ## Autonomous Chooser
        self.m_autoChooser = SendableChooser()
        self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        ## Default Commands
        self.drive.setDefaultCommand( DriveByStick(
            self.drive,
            self.m_driver1.getLeftX,
            self.m_driver1.getLeftY,
            lambda: ( self.m_driver1.getRightTriggerAxis() - self.m_driver1.getLeftTriggerAxis() )
        ) )

        ## Put Data on SmartDashboard
        SmartDashboard.putData( self.drive )
        SmartDashboard.putData( self.drive.getDefaultCommand() )

        ## Controls
        # self.m_driver1.a().whileTrue( self.rightX )

    def getAutonomousCommand(self) -> Command:
        chooserValue = self.m_autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()