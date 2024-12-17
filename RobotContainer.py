from wpilib import SendableChooser, SmartDashboard

from wpimath.units import degreesToRotations

# from pathplannerlib.auto import AutoBuilder

# from phoenix6.hardware import Pigeon2

from commands2 import Command
from commands2.button import CommandXboxController
import commands2.cmd as cmd

from commands.DriveByStick import DriveByStick
from subsystems.SwerveDrive.SwerveDrive import SwerveDrive
from subsystems.SwerveDrive.SwerveModule import SwerveModule
from subsystems.SwerveDrive.CustomPigeon2 import CustomPigeon2

class RobotContainer:
    # Variable Declaration
    m_autoChooser:SendableChooser = None

    def __init__(self):
        # Driver Controller
        self.m_driver1 = CommandXboxController( 0 )

        ## Declare Subsystems
        #Drive
        gyro = CustomPigeon2( 9, 'canivore1')
        swerve_modules = [
            SwerveModule("FL", 7, 8, 18, -0.235352 ),#degreesToRotations(97.471)),#
            SwerveModule("FR", 1, 2, 12, -0.486572 ),#degreesToRotations(5.361)),#
            SwerveModule("BL", 5, 6, 16, -0.673584 ),#degreesToRotations(298.828)),#
            SwerveModule("BR", 3, 4, 14, -0.338 )#degreesToRotations(60.557)),#
        ]
        
        self.drive = SwerveDrive( 'canivore1', swerve_modules, gyro )

        ## Commands


        ## Autonomous Chooser
        self.m_autoChooser = SendableChooser()#AutoBuilder.buildAutoChooser()
        # self.m_autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.m_autoChooser )

        ## Default Commands
        self.drive.setDefaultCommand( DriveByStick(
            self.drive,
            self.m_driver1.getLeftY,
            self.m_driver1.getLeftX,
            lambda: ( self.m_driver1.getLeftTriggerAxis() - self.m_driver1.getRightTriggerAxis() )
        ) )

        ## Put Data on SmartDashboard
        SmartDashboard.putData( self.drive )
        # SmartDashboard.putData( self.drive.getDefaultCommand() )

        ## Controls
        self.m_driver1.start().onTrue(cmd.runOnce(self.drive.sync_gyro))
        # self.m_driver1.a().whileTrue( self.rightX )
    
    def sync_gyro_to_vision(self):
        self.drive.sync_gyro()

    def getAutonomousCommand(self) -> Command:
        chooserValue = self.m_autoChooser.getSelected()
        if type(chooserValue) == Command:
            return chooserValue
        else:
            return cmd.none()