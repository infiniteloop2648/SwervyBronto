package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class PIDArmWrist extends CommandBase{
    double armSetpoint;
    double wristSetpoint;
    Arm arm;
    Wrist wrist;
    
    public PIDArmWrist(double armSetpoint, double wristSetpoint, Arm arm, Wrist wrist){
        this.armSetpoint = armSetpoint;
        this.wristSetpoint = wristSetpoint;
        this.arm = arm;
        this.wrist = wrist;

        addRequirements(arm, wrist);
    }

    @Override
    public void initialize() {
        wrist.enable();
        wrist.setGoal(wristSetpoint);
        arm.enable();
        arm.setGoal(armSetpoint);
    }

    @Override
    public boolean isFinished(){
        return wrist.getController().atGoal() && arm.getController().atGoal();
    }

    @Override
    public void end(boolean interrupted){
        wrist.disable();
        arm.disable();
    }
}
