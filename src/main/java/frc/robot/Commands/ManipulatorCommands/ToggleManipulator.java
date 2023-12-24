package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class ToggleManipulator extends CommandBase{
    
    private Manipulator manipulator;

    public ToggleManipulator(Manipulator manipulator){
        this.manipulator = manipulator;
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.ToggleManipulator();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
