package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.WristConstants;

public class Wrist extends ProfiledPIDSubsystem{
    
    private MotorController wristMotor;
    private Encoder wristEncoder;

    private ArmFeedforward wristFeedForward;

    private DigitalInput wristSwitch;

    private DoubleSupplier armPosition;

    public Wrist(DoubleSupplier armPosition){
        super(new ProfiledPIDController(WristConstants.kWristP, WristConstants.kWristI, WristConstants.kWristD,
        new TrapezoidProfile.Constraints(WristConstants.kWristMaxVelocityPerSecond, WristConstants.kWristMaxAccelerationPerSecond2)));
        getController().setTolerance(0.0);
        wristMotor = new WPI_VictorSPX(WristConstants.kWrist);

        wristFeedForward = new ArmFeedforward(WristConstants.kWristFeedS, WristConstants.kWristFeedG, WristConstants.kWristFeedV, WristConstants.kWristFeedA);

        wristEncoder = new Encoder(WristConstants.kSourceChannelWristEncoder1, WristConstants.kSourceChannelWristEncoder2);
        wristEncoder.setDistancePerPulse(WristConstants.kWristEncoderMultiplier);
        wristEncoder.setSamplesToAverage(10);

        wristSwitch = new DigitalInput(WristConstants.kSourceChannelWristSwitch);

        this.armPosition = armPosition;
    }

    @Override
    public void periodic(){
        super.periodic();
        
        getController().setGoal(getController().getSetpoint());

        /*if(!wristSwitch.get()){
            wristEncoder.reset();
        }*/
        
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        wristMotor.setVoltage(-(output + wristFeedForward.calculate(setpoint.position+armPosition.getAsDouble(), setpoint.velocity)));
    }

    @Override
    protected double getMeasurement() {

        return wristEncoder.getDistance()+WristConstants.kWristStowedAngle;
    }

    public void powerWrist(double throttle){
        if(!wristSwitch.get()){
            System.out.println("manual control");
            wristMotor.set(MathUtil.clamp(throttle, 0, 1));
        }else{
            wristMotor.set(throttle);
        }
         
    }

    public void voltsWrist(double throttle){
        wristMotor.setVoltage(throttle);
    }

    public boolean atSetpoint(){
        return getController().atSetpoint();
    }

    public double getWristAngle(){
        return Units.radiansToDegrees(wristEncoder.getDistance()+WristConstants.kWristStowedAngle);
    }

    public double getMotorSet() {
        return wristMotor.get();
    }

}
