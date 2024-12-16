import typing, threading
import math

from commands2 import Subsystem
from wpilib import RobotState, SmartDashboard, Field2d, DriverStation
from wpilib.shuffleboard import Shuffleboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Translation2d, Pose2d, Pose3d, Transform2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState, SwerveDrive4Odometry, ChassisSpeeds
from wpimath.system.plant import DCMotor
from wpimath.units import lbsToKilograms

from ntcore import NetworkTable, NetworkTableInstance, _now
from ntcore.util import ntproperty

from phoenix6.hardware import Pigeon2

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants, ModuleConfig

from subsystems.SwerveModule import SwerveModule, SwerveModuleConstants

class SwerveDriveConstants:
    kWeightLbs = 120.0
    kMaxSpeed = 4.4
    kRotationSpeed = math.pi

class SwerveDrive(Subsystem):
    # Variable Declaration
    __modules:typing.Tuple[ SwerveModule, SwerveModule, SwerveModule, SwerveModule ] = None
    __gyro:Pigeon2 = None
    __kinematics:SwerveDrive4Kinematics = None
    __odometry:SwerveDrive4Odometry = None
    __visionOdometry:SwerveDrive4PoseEstimator = None
    __logging:NetworkTable = None

    # Settings
    __DriveFieldRelative = ntproperty( "/Settings/Driver1/FieldRelative", True )
    __DriveMaxSpeedPercent = ntproperty( "/Settings/Driver1/MaxSpeedPercent", 1.0 )
    __DriveMaxRotationPercent = ntproperty( "/Settings/Driver1/MaxRotationPercent", 1.0 )

    __setpoint:ChassisSpeeds = None
    __odometryLock = False

    # Initialization
    def __init__(self) -> None:
        self.setName( "SwerveDrive" )

        self.__modules = [
            SwerveModule( 0, 7, 8, 18, -0.235352 ), # 97.471 ),
            SwerveModule( 1, 1, 2, 12, -0.486572 ), #5.361 ),
            SwerveModule( 2, 5, 6, 16, -0.673584 ), #298.828 ),
            SwerveModule( 3, 3, 4, 14, -0.338 ) #60.557 )
        ]

        self.__gyro = Pigeon2( 9, "canivore1" )

        self.__kinematics = SwerveDrive4Kinematics(
            Translation2d( 0.2667, 0.2667 ),
            Translation2d( 0.2667, -0.2667 ),
            Translation2d( -0.2667, 0.2667 ),
            Translation2d( -0.2667, -0.2667 )
        )

        self.__resetOdometry( Pose2d() )

        self.stop()

        # Dashboards
        Shuffleboard.getTab( "SwerveDrive" ).add( "SwerveDrive", self )
        self.__field = Field2d()
        SmartDashboard.putData("Field", self.__field)

        self.__logging = NetworkTableInstance.getDefault().getTable("/Logging/SwerveDrive")
        self.__outGyro = NetworkTableInstance.getDefault().getStructTopic("/RealOutputs/SwerveDrive/Gyro",Rotation2d).publish()
        self.__outOdometry = NetworkTableInstance.getDefault().getStructTopic("/RealOutputs/SwerveDrive/Odometry",Pose2d).publish()
        self.__outVisionOdometry = NetworkTableInstance.getDefault().getStructTopic("/RealOutputs/SwerveDrive/OdometryPlusVision",Pose2d).publish()
        self.__outChassisSpeedsActual = NetworkTableInstance.getDefault().getStructTopic("/RealOutputs/SwerveDrive/ChassisSpeeds/Actual", ChassisSpeeds).publish()
        self.__outChassisSpeedsTarget = NetworkTableInstance.getDefault().getStructTopic("/RealOutputs/SwerveDrive/ChassisSpeeds/Target", ChassisSpeeds).publish()
        self.__outSwerveModuleStateActual = NetworkTableInstance.getDefault().getStructArrayTopic("/RealOutputs/SwerveDrive/SwerveModuleStates/Actual", SwerveModuleState).publish()
        self.__outSwerveModuleStateTarget = NetworkTableInstance.getDefault().getStructArrayTopic("/RealOutputs/SwerveDrive/SwerveModuleStates/Target", SwerveModuleState).publish()
       
        # Path Planner
        # robotConfig = RobotConfig.fromGUISettings()
        moduleConfig = ModuleConfig(
            wheelRadiusMeters = SwerveModuleConstants.Drive.kWheelRadius,
            maxDriveVelocityMPS = SwerveDriveConstants.kMaxSpeed,
            wheelCOF = 1.0,
            driveMotor = DCMotor.NEO(1),
            driveCurrentLimit = 40.0,
            numMotors = 1
        )
        robotConfig = RobotConfig(
            massKG = lbsToKilograms( SwerveDriveConstants.kWeightLbs ),
            MOI = 1.0,
            moduleConfig = moduleConfig,
            moduleOffsets = self.__kinematics.getModules(),
            trackwidthMeters = None
        )
        AutoBuilder.configure(
            pose_supplier = self.__odometry.getPose,
            reset_pose = self.__odometry.resetPose,
            robot_relative_speeds_supplier = self.getChassisSpeeds,
            output = lambda speeds, feedforwards: self.runChassisSpeeds(speeds),
            controller = PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),
            robot_config = robotConfig,
            should_flip_path = self.shouldFlipPath,
            drive_subsystem = self
        )

    def shouldFlipPath(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def resetGyro(self) -> None:
        # Thread Safe Function
        def resetGyroThread() -> None:
            # Get The Current Pose Data
            currentPose = self.__visionOdometry.getEstimatedPosition()
            self.__gyro.set_yaw( currentPose.rotation().degrees(), 0.5 )
            self.__resetOdometry( currentPose )
            self.__odometryLock = False
            print( "OdometryUnlocked!" )

        # Odometry Lock
        self.__odometryLock = True
        print( "OdometryLock!" )
        threading.Thread( target=lambda: resetGyroThread() ).start()

    def __resetOdometry(self, pose:Pose2d) -> None:
        self.__odometry = SwerveDrive4Odometry(
            self.__kinematics,
            self.getRobotAngle(),
            self.__getModulePositions(),
            pose
        )
        self.__visionOdometry = SwerveDrive4PoseEstimator(
            self.__kinematics,
            self.getRobotAngle(),
            self.__getModulePositions(),
            pose
        )

    # Periodic Loop
    def periodic(self) -> None:
        # Input Logging
        self.__logging.putValue( "Gyro/yaw_d", self.__gyro.get_yaw().value )
        self.__logging.putValue( "Gyro/pitch_d", self.__gyro.get_pitch().value  )
        self.__logging.putValue( "Gyro/roll_d", self.__gyro.get_roll().value  )
        self.__logging.putBoolean( "OdometryLock", self.__odometryLock )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        
        # Run SwerveModules
        self.__modules[0].run()
        self.__modules[1].run()
        self.__modules[2].run()
        self.__modules[3].run()
        
        # Update Odometry
        pose = self.__odometry.getPose()
        vPose = self.__visionOdometry.getEstimatedPosition()

        if not self.__odometryLock:
            pose = self.__odometry.update(
                self.getRobotAngle(),
                self.__getModulePositions()
            )

            # Update Vision Odometry
            vPose = self.__visionOdometry.update(
                self.getRobotAngle(),
                self.__getModulePositions()
            )
        
        # Dashboarding -- Updated Odometry to only use Blue Relative
        self.__field.setRobotPose( pose )
        self.__field.getObject( "BlueVisionPose" ).setPose( vPose )
        # match DriverStation.getAlliance():
        #     case DriverStation.Alliance.kBlue:
        #         self.__field.setRobotPose( pose )
        #         self.__field.getObject( "Vision" ).setPose( vPose )
        # if self.shouldFlipPath():
        #     rPose = Pose2d( x=16.523 - pose.X(), y=8.013 - pose.Y(), angle= pose.rotation().radians() - math.pi )
        #     rvPose = Pose2d( x=16.523 - vPose.X(), y=8.013 - vPose.Y(), angle= vPose.rotation().radians() - math.pi )
        #     self.__field.getObject( "RedPose" ).setPose( vPose )
        #     self.__field.getObject( "RedVisionPose" ).setPose( rvPose )
       
        # Output Logging
        ntTime = _now()
        self.__outGyro.set( self.__gyro.getRotation2d(), ntTime )
        self.__outOdometry.set( pose, ntTime )
        self.__outVisionOdometry.set( vPose, ntTime )
        self.__outSwerveModuleStateActual.set( self.__getModuleStates(), ntTime )
        self.__outSwerveModuleStateTarget.set( self.__setpointStates, ntTime )
        self.__outChassisSpeedsActual.set( self.getChassisSpeeds(), ntTime )
        self.__outChassisSpeedsTarget.set( self.__setpoint, ntTime )

    # Simulation Periodic Loop
    def simulationPeriodic(self) -> None:
        # Run SwerveModules
        self.__modules[0].runSim()
        self.__modules[1].runSim()
        self.__modules[2].runSim()
        self.__modules[3].runSim()

        # Update Gyro
        moduleStates = self.__getModuleStates()
        actualSpeed = self.__kinematics.toChassisSpeeds( moduleStates )
        yawPer20ms = actualSpeed.omega_dps * 0.02
        self.__gyro.sim_state.add_yaw( yawPer20ms )

    # Stop the Subsystem
    def stop(self) -> None:
        self.runChassisSpeeds( ChassisSpeeds( 0.0, 0.0, 0.0 ) )
 
    def getRobotAngle(self) -> Rotation2d:
        rotateBy = 180.0 if self.shouldFlipPath() else 0.0
        return self.__gyro.getRotation2d().rotateBy( Rotation2d.fromDegrees(rotateBy) )

    # Run By Percentage
    def runPercentInputs(self, x:float, y:float, omega:float) -> None:
        # Range Tolerances
        x = min( max( x, -1.0 ), 1.0 )
        y = min( max( y, -1.0 ), 1.0 )
        omega = min( max( omega, -1.0 ), 1.0 )

        xSpeed = x * self.__DriveMaxSpeedPercent * SwerveDriveConstants.kMaxSpeed
        ySpeed = y * self.__DriveMaxSpeedPercent * SwerveDriveConstants.kMaxSpeed
        omegaSpeed = omega * self.__DriveMaxRotationPercent * SwerveDriveConstants.kRotationSpeed

        cSpeed = (
            ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, omegaSpeed, self.__gyro.getRotation2d() ) # self.getRobotAngle() )
            if self.__DriveFieldRelative
            else ChassisSpeeds( xSpeed, ySpeed, omegaSpeed )
        )
        self.runChassisSpeeds( cSpeed )

    # Run By Chassis Speeds
    def runChassisSpeeds(self, chassisSpeeds:ChassisSpeeds) -> None:
        self.__setpoint = ChassisSpeeds.discretize( chassisSpeeds, 0.02 )
        self.__setpointStates = self.__kinematics.toSwerveModuleStates( self.__setpoint )
        self.runModuleStates( self.__setpointStates )

    # Run By SwerveModuleStates
    def runModuleStates(self, swerveStates:typing.Tuple[ SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState ]) -> None:
        newStates = SwerveDrive4Kinematics.desaturateWheelSpeeds( swerveStates, SwerveDriveConstants.kMaxSpeed )
        self.__modules[0].setState(newStates[0])
        self.__modules[1].setState(newStates[1])
        self.__modules[2].setState(newStates[2])
        self.__modules[3].setState(newStates[3])

    def getOdometry(self) -> SwerveDrive4PoseEstimator:
        if self.__odometryLock:
            raise "Odometry Lock In Place" 
        return self.__visionOdometry

    def __getModulePositions(self) -> typing.Tuple[ SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        return [
            self.__modules[0].getPosition(),
            self.__modules[1].getPosition(),
            self.__modules[2].getPosition(),
            self.__modules[3].getPosition()
        ]
    
    def __getModuleStates(self) -> typing.Tuple[ SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return [
            self.__modules[0].getState(),
            self.__modules[1].getState(),
            self.__modules[2].getState(),
            self.__modules[3].getState()
        ]

    def getChassisSpeeds(self) -> ChassisSpeeds:
        return self.__kinematics.toChassisSpeeds( self.__getModuleStates() )

    def getMaxSpeed(self) -> float:
        return SwerveDriveConstants.kMaxSpeed
    
    def getMaxRotation(self) -> float:
        return SwerveDriveConstants.kRotationSpeed

    def getDriverMaxSpeed(self) -> float:
        return SwerveDriveConstants.kMaxSpeed * self.__DriveMaxSpeedPercent
    
    def getDriverMaxRotation(self) -> float:
        return SwerveDriveConstants.kRotationSpeed * self.__DriveMaxRotationPercent

    def getFieldRelative(self) -> bool:
        return self.__DriveFieldRelative
    
    def setFieldRelative(self, isFieldRelative:bool) -> None:
        self.__DriveFieldRelative = isFieldRelative

    def toggleFieldRelative(self) -> None:
        self.__DriveFieldRelative = not self.__DriveFieldRelative