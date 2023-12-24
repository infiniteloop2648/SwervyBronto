package frc.robot.Commands.WristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class PIDWrist extends CommandBase{
    private Wrist wrist;
    private double setpoint;
    public PIDWrist(double setpoint, Wrist wrist){
        this.wrist = wrist;
        this.setpoint = setpoint;

        addRequirements(wrist);
    }

    @Override
    public void initialize(){
        wrist.enable();
        wrist.setGoal(setpoint);
    }


    @Override
    public boolean isFinished() {
        return wrist.getController().atGoal();
    }

    @Override
    public void end(boolean interrupted){
        wrist.disable();
    }
    
}
