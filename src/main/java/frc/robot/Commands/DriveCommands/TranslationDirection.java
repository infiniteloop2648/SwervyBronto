package frc.robot.Commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TranslationDirection extends PIDCommand {

    public TranslationDirection( DoubleSupplier yVelocity, DriveSubsystem m_robot_drive){
        super(
            new PIDController(AutoConstants.kPXController, 0, 0),
            // Close loop on heading
            m_robot_drive::getPoseX,
            // Set reference to target
            -2.0,
            // Pipe output to turn robot
            (output) -> m_robot_drive.drive(-output, yVelocity.getAsDouble(), 0.0, true, true),
            // Require the drive
            m_robot_drive);

            getController()
            .setTolerance(0.0, 0.0);
            addRequirements(m_robot_drive);
    }

    @Override
    public boolean isFinished(){
        return getController().atSetpoint();
    }
}
