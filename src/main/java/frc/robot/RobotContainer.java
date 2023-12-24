// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.OnePiece;
import frc.robot.Commands.PIDArmWrist;
import frc.robot.Commands.ArmCommands.PIDArm;
import frc.robot.Commands.DriveCommands.CardinalDirections;
import frc.robot.Commands.ManipulatorCommands.CloseManipulator;
import frc.robot.Commands.ManipulatorCommands.OpenManipulator;
import frc.robot.Commands.WristCommands.ManualWrist;
import frc.robot.Commands.WristCommands.PIDWrist;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    private final Arm arm = new Arm();

    private final Wrist wrist = new Wrist(arm::armPosition);

    private final Manipulator manipulator = new Manipulator();

    // The driver's controller
    XboxController primary = new XboxController(OIConstants.kDriverControllerPort);
    XboxController secondary = new XboxController(OIConstants.kOperatorControllerPort);

    List<PathPlannerTrajectory> testPath = PathPlanner.loadPathGroup("Test Path", new PathConstraints(3.5, 1.0));
    List<PathPlannerTrajectory> dummyPathGroup = PathPlanner.loadPathGroup("Dummy Bump Path", new PathConstraints(3.0, 1.0));
    List<PathPlannerTrajectory> BumpPathGroup = PathPlanner.loadPathGroup("Bump Path", new PathConstraints(3.0, 1));
    List<PathPlannerTrajectory> ClearPathGroup = PathPlanner.loadPathGroup("Two Piece Path", new PathConstraints(3.0, 1.0));
    List<PathPlannerTrajectory> StationPathGroup = PathPlanner.loadPathGroup("Charge Station Path", new PathConstraints(3.0, 1.0));

    UsbCamera camera0 = CameraServer.startAutomaticCapture();

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public static HashMap<String, Command> eventMap = new HashMap<>();    

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    private SwerveAutoBuilder autoBuilder;

    private Compressor compressor;

    public RobotContainer() {
        

        eventMap.put("level3Position",
            Commands.parallel(
                new PIDArmWrist(ArmConstants.kArmLevel3Angle, WristConstants.kWristLevel3Angle, arm, wrist)));
        eventMap.put("moveWrist", 
            new PIDArmWrist(ArmConstants.kArmLevel3Angle, WristConstants.kWristLevel3Angle, arm, wrist));
        eventMap.put("open",
             new OpenManipulator(manipulator));
        eventMap.put("stow",
            Commands.parallel(new PIDArm(ArmConstants.kArmInitialPosition, arm),
            new PIDWrist(WristConstants.kWristStowedAngle, wrist)));
        eventMap.put("groundPosition", 
            new PIDArmWrist(ArmConstants.kArmFloorAngle, WristConstants.kWristFloorAngle, arm, wrist));
        eventMap.put("pickup", 
            new CloseManipulator(manipulator));
        eventMap.put("level3PositionCube",
        Commands.parallel(
            new PIDArmWrist(ArmConstants.kArmLevel3Angle, WristConstants.kWristLevel3Angle, arm, wrist)));


        autoBuilder = new SwerveAutoBuilder(
            m_robotDrive::getPose, // Pose2d supplier
            m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(AutoConstants.kPXController, 0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(AutoConstants.kPThetaController, 0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
            m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
        );
        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));

        PathPlannerServer.startServer(5811);

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        configureBindings();
        shuffleboardSetup();

    }

    

    private void configureBindings() {

        new POVButton(primary, -1).toggleOnFalse(new CardinalDirections(
            () -> -MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband),
            primary::getPOV,
            m_robotDrive).until(() -> MathUtil.applyDeadband(primary.getRightX()+primary.getRightY(), OIConstants.kDriveDeadband)!=0));
        
        new JoystickButton(primary, Button.kY.value).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        
        new JoystickButton(primary, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
 
        new Trigger(() -> {
            if(arm.isEnabled() && Math.abs(secondary.getLeftY()) > .05) {
                return true;
            } else {
                return false;
            }
        }).onTrue(Commands.runOnce(() -> {
            //I do nothing
        }, arm));

        new Trigger(() -> {
            if(wrist.isEnabled() && Math.abs(secondary.getRightY()) > .05) {
                return true;
            } else {
                return false;
            }
        }).onTrue(Commands.parallel(
            new PrintCommand("Cancel Active Wrist Command"),
            Commands.runOnce(() -> {}, wrist)
        ));
        
        arm.setDefaultCommand(Commands.runEnd(
            () -> {
                if(!arm.isEnabled()){
                    if(Math.abs(secondary.getLeftY()) > .05) {
                        if(arm.isFeedForwarding()) {
                            arm.toggleFeedForward();
                        }

                        arm.setPower(-secondary.getLeftY());
                    } else {
                        if(!arm.isFeedForwarding()) {
                            arm.toggleFeedForward();
                        }
                    }
                }
            }, 
            () -> {}, 
            arm));


        wrist.setDefaultCommand(
            Commands.parallel(
                new PrintCommand("Wrist Default Command Triggered"),
                new ManualWrist(wrist, secondary::getRightY)
            ));

        new JoystickButton(primary, Button.kLeftBumper.value).onTrue(Commands.runOnce(
            manipulator::ToggleManipulator, manipulator));

        new POVButton(secondary, 180).onTrue(
            new PIDArmWrist(ArmConstants.kArmFloorAngle, 0.0, arm, wrist)
        ); 

        new JoystickButton(secondary, Button.kA.value).onTrue(
            new PIDArm(ArmConstants.kArmInitialPosition, arm)
        );

        new POVButton(secondary, 270).onTrue(
            Commands.parallel(
                new PIDArm(ArmConstants.kArmLevel2Angle, arm)
        ));

        new POVButton(secondary, 0).onTrue(
            new PIDArm(ArmConstants.kArmLevel3Angle, arm)
        );

        new POVButton(secondary, 90).onTrue(
            new PIDArmWrist(ArmConstants.kArmHumanPlayerStationAngle, Units.degreesToRadians(0.0), arm, wrist)
        ); 
        
    }

    private void shuffleboardSetup() {
        ShuffleboardTab tab = Shuffleboard.getTab("Encoder Data");

        tab.addNumber("Arm Encoder Radians", arm::armPosition);
        tab.addNumber("Arm Last Set Voltage", arm::getLastSetVoltage);

        tab.addBoolean("Arm Enabled?", arm::isEnabled);
        tab.addBoolean("Arm Feed Forward State", arm::isFeedForwarding);
        tab.addNumber("Arm Feed Forward Volts", arm::getLastSetVoltage);
        tab.addNumber("wrist angle", wrist::getWristAngle);

        tab.addNumber("Wrist Motor Set Value", wrist::getMotorSet);
        tab.addBoolean("Wrist PID Enabled", wrist::isEnabled);
        tab.addBoolean("wrist atsetpoint", wrist::atSetpoint);

        tab.addNumber("navx angle", m_robotDrive::getGyroAngle);
        tab.addNumber("hahahahaha", m_robotDrive::getPoseX);
        
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Stuff");
        m_chooser.setDefaultOption("Nothing", null);
        m_chooser.addOption("Test Path", autoBuilder.fullAuto(testPath));
        m_chooser.addOption("Dummy Bump Path", autoBuilder.fullAuto(dummyPathGroup));
        m_chooser.addOption("2 Piece Bump", autoBuilder.fullAuto(BumpPathGroup));
        m_chooser.addOption("Open 2 Piece", autoBuilder.fullAuto(ClearPathGroup));
        m_chooser.addOption("Charge Station Auto", autoBuilder.fullAuto(StationPathGroup));
        m_chooser.addOption("One Piece Back", new OnePiece(m_robotDrive, arm, wrist, manipulator));


        autoTab.add("Auto Chooser", m_chooser);
    }

    public Command getAutonomousCommand() {
        
        
        return m_chooser.getSelected();
    }
}
