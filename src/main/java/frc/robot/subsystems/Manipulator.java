package frc.robot.subsystems;

import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase{
    
    private DoubleSolenoid claw;
    
    public Manipulator(){
        claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ManipulatorConstants.kClawPCMOut, ManipulatorConstants.kClawPCMIn);
        claw.set(Value.kReverse);
    }

    public void CloseManipulator(){
        claw.set(DoubleSolenoid.Value.kReverse);
    }

    public void OpenManipulator(){
        claw.set(DoubleSolenoid.Value.kForward);
    }

    public void ToggleManipulator(){
        claw.toggle();
    }
}
