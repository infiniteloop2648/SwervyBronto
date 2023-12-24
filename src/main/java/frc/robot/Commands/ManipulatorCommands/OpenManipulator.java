package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class OpenManipulator extends CommandBase{
    
    private Manipulator manipulator;

    public OpenManipulator(Manipulator manipulator){
        this.manipulator = manipulator;
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.OpenManipulator();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
