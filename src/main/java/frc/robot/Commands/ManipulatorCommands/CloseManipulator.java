package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class CloseManipulator extends CommandBase{
    
    private Manipulator manipulator;

    public CloseManipulator(Manipulator manipulator){
        this.manipulator = manipulator;
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.CloseManipulator();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
