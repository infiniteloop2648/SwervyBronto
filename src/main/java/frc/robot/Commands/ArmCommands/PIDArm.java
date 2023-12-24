package frc.robot.Commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PIDArm extends CommandBase{
    private Arm arm;
    private double setpoint;
    public PIDArm(double setpoint, Arm arm){
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.enable();
        arm.setGoal(setpoint);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished() {
        return arm.getController().atGoal();
    }

    @Override
    public void end(boolean interrupted){
        arm.disable();
    }
}
