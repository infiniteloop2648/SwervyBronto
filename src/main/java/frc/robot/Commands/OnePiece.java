package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ManipulatorCommands.CloseManipulator;
import frc.robot.Commands.ManipulatorCommands.OpenManipulator;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class OnePiece extends SequentialCommandGroup{

    public OnePiece(DriveSubsystem m_robotDrive, Arm arm, Wrist wrist, Manipulator manipulator){

        addCommands(
        new CloseManipulator(manipulator),
        new PIDArmWrist(ArmConstants.kArmLevel3Angle, Units.degreesToRadians(40.0), arm, wrist),
        new RunCommand(() -> m_robotDrive.drive(0.25, 0.0, 0.0, true, true) ).withTimeout(0.75), 
        new PIDArmWrist(ArmConstants.kArmLevel3Angle, Units.degreesToRadians(0.0), arm, wrist),
        new OpenManipulator(manipulator),
        Commands.parallel(new RunCommand(() -> m_robotDrive.drive(0.25, 0.0, 0.0, true, true)).withTimeout(3.0), new PIDArmWrist(0, 0, arm, wrist))
        );

    }
    
}
