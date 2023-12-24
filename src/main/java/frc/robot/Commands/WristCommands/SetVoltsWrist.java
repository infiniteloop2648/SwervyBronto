package frc.robot.Commands.WristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;


public class SetVoltsWrist extends CommandBase{

    private double throttle;
    private Wrist wrist;
    public SetVoltsWrist(Wrist wrist, double throttle){
        this.throttle = throttle;
        this.wrist = wrist;
        this.addRequirements(wrist);
    }

    @Override
    public void execute(){
        wrist.voltsWrist(throttle);
        
    }
    
}
