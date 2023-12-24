package frc.robot.Commands.WristCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;


public class ManualWrist extends CommandBase{

    private Supplier<Double> throttleSupplier;

    private Wrist wrist;
    public ManualWrist(Wrist wrist, Supplier<Double> throttleSupplier){
        this.throttleSupplier = throttleSupplier;
        this.wrist = wrist;
        this.addRequirements(wrist);
    }

    @Override
    public void execute(){
        wrist.powerWrist(throttleSupplier.get());
        
    }
    
}
