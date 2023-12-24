package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*
 * This subsystem is heavily based on 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/profilepid-subsystems-commands.html
 * 
 * Assuming values for specific subcomponents are correct (PID, FeedForwarder, TrapezoidProfile, etc.), 
 * this should be able to get the arm to a particular setpoint (specified in RADIANS) when asked
 * using the setGoal(double goal) method that is exposed as part of the parent class (ProfiledPIDSubsystem). 
 * 
 * It is VERY IMPORTANT that all units, for the encoder, for the goals, for the volt units, etc. are RADIAN
 * based. If they are not, THIS. WILL. NOT. WORK. The ArmFeedforward class EXPECTS RADIANS. If it gets degrees
 * (or some other non-descript unit) the calculation will be wrong, and the resulting feed forward voltage
 * will be wrong. All other elements (like the PID Controller and the Encoder) take on the units
 * provided to them, so the only real concern is the Feed Forward here, which is dictating
 * the units for all of the other component pieces. 
 */
public class Arm extends ProfiledPIDSubsystem {

    private CANSparkMax armMotor;

    private ArmFeedforward armFeedForward;

    private Encoder armEncoder;

    private DigitalInput armSwitch;

    private LinearFilter measurementFilter;

    private boolean isFeedForwarding;

    private double currentMeasurement;

    //TODO Remove me
    public double lastSetVoltage;
    public double pidVoltage;

    public Arm(){
        /*
         * The parent class (ProfiledPIDSubsystem) must have a ProfiledPIDController to
         * work with, the PID values can be gleaned from SysID (hopefully) while the 
         * Constraints for the TrapezoidProfile we're probably going to have to guess at.
         * The parent class also requires an initial position, because our starting configuration
         * should always be more or less the same, this value will probably stay 0, and we'll
         * scale our desired positional values accordingly
         */
        super(
            new ProfiledPIDController(
                ArmConstants.kArmP, 
                ArmConstants.kArmI,
                ArmConstants.kArmD, 
                new TrapezoidProfile.Constraints(
                    ArmConstants.kArmMaxVelocitySecond, 
                    ArmConstants.kArmMaxAccelRadPerSecond2)),
            ArmConstants.kArmInitialPosition
        );

        this.getController().setTolerance(Units.degreesToRadians(2.5));

        armMotor = new CANSparkMax(ArmConstants.kArm, MotorType.kBrushless);
        armMotor.setInverted(true);
        
        /*
         * Again, the units of literally everything MUST be in RADIANS in 
         * order for things to work. The multiplier for the Encoder needs to take
         * this into account. 
         */
        armEncoder = new Encoder(ArmConstants.kSourceChannelArmEncoder1, ArmConstants.kSourceChannelArmEncoder2);
        armEncoder.setDistancePerPulse(ArmConstants.kArmEncoderMultiplier);
        armEncoder.reset();

        measurementFilter = LinearFilter.movingAverage(10);

        armFeedForward = new ArmFeedforward(
            ArmConstants.kArmFeedS, 
            ArmConstants.kArmFeedG, 
            ArmConstants.kArmFeedV, 
            ArmConstants.kArmFeedA);

        /*
         * We start out with the underlying ProfiledPIDController disabled, this 
         * ensures no funny business when trying to manually control the robot.
         * 
         * This may need to change if we get this well tuned and begin using
         * set positions for arm motion.
         * 
         * We also set the initial goal here, which is just to keep the arm at its resting position.
         * This may not be ideal, we may want the resting position to be up slightly, just
         * to ensure the motor doesn't try to fight to force the arm up against it's mechanical
         * stops. 
         */
        setGoal(ArmConstants.kArmInitialPosition);

        armSwitch = new DigitalInput(ArmConstants.kSourceChannelArmSwitch);

        isFeedForwarding = false;
    }

    @Override
    public void periodic(){
        super.periodic();
        /*if(!armSwitch.get()){
            armEncoder.reset();
        }*/

        currentMeasurement = measurementFilter.calculate(armEncoder.getDistance()+ArmConstants.kArmInitialPosition);

        if(isFeedForwarding && !isEnabled()) {
            lastSetVoltage = armFeedForward.calculate(armPosition(), 0);
            armMotor.setVoltage(lastSetVoltage);
        }
        

    }

    public void toggleFeedForward() {
        isFeedForwarding = !isFeedForwarding;
    }

    public boolean isFeedForwarding() {
        return isFeedForwarding;
    }

    public void feedForward(double position, double velocity) {
        lastSetVoltage = armFeedForward.calculate(position, velocity);
        armMotor.setVoltage(lastSetVoltage);
    }

    public void setPower(double throttle){
        //This if statement is just insurance just in case something dumb happens,
        //to make sure that when the PID is running, there is no manual control 

        armMotor.set(-throttle);

        if(!isEnabled()) {
            if(!armSwitch.get()){
                armMotor.set(MathUtil.clamp(throttle, 0, 1)/* + armFeedForward.calculate()*/);
            }else if(armPosition()> Units.degreesToRadians(4)){
                armMotor.set(MathUtil.clamp(throttle, -1, 0)/* + armFeedForward.calculate()*/);
            }else{
                armMotor.set(throttle);
            }
        }
    }

    @Override
    /*
     * This is part of ProfiledPIDSubsystem, when the underlying ProfiledPIDController
     * is enabled the resultant output and current TrapezoidProfile State is provided
     * for us to perform actions with. 
     * 
     * All we are doing is calculating the appropriate FeedForward using the current 
     * desired State information, and applying both the output and the feed forward result 
     * to the motor as a voltage. 
     * 
     * TrapezoidProfile provides states over time that vary, so rather than directing
     * a PID Controller to immediately go to the exact setpoint we want, it gradually
     * over time approaches the desired setpoint by specifying smaller, incremental changes. 
     * 
     * I assume the incrementing happens based on what the max velocity and max acceleration
     * values are. But I'm not 100% on this. It makes some sense in my head that those values
     * would be involved, along with the update period of the robot's main thread (20ms). But again,
     * ¯\_(ツ)_/¯
     */
    protected void useOutput(double output, State setpoint) {
        
        lastSetVoltage = armFeedForward.calculate(
            setpoint.position, setpoint.velocity);
            
        pidVoltage = output;

        armMotor.setVoltage(output + lastSetVoltage);
        
    }

    @Override
    /*
     * Provides the underlying ProfilePIDController with the measurement value used
     * for calculation. I cannot stress this enough, RADIANS!
     */
    protected double getMeasurement() {
        return armPosition();
    }

    public double armPosition(){
        return currentMeasurement;
    }

    public double getArmSetpoint(){
        return getController().getSetpoint().position;
    }

    public double getArmVelocitySetpoint(){
        return getController().getSetpoint().velocity;
    }

    //TODO Remove me
    public double getLastSetVoltage() {
        return lastSetVoltage;
    }

    //TODO Remove me
    public double getLastPIDVoltage() {
        return pidVoltage;
    }

    public boolean atSetpoint(){
        return getController().atSetpoint();
    }
}
