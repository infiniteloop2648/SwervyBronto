package frc.robot.Commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class CardinalDirections extends PIDCommand {

    public CardinalDirections(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angleSetpoint, DriveSubsystem m_robot_drive){
        super(
            new PIDController(AutoConstants.kPHeadingController, 0, AutoConstants.kDHeadingController),
            // Close loop on heading
            m_robot_drive::getHeading,
            // Set reference to target
            angleSetpoint.getAsDouble(),
            // Pipe output to turn robot
            (output) -> m_robot_drive.drive(xVelocity.getAsDouble(), yVelocity.getAsDouble(), output, true, true),
            // Require the drive
            m_robot_drive);

            getController().enableContinuousInput(-180, 180);

            getController()
            .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
            addRequirements(m_robot_drive);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public boolean isFinished(){
        return getController().atSetpoint();
    }
}
