// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.6;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 2.4; // radians per second
        public static final double kMagnitudeSlewRate = 3.2; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5-1.75*2);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(32.3-1.75*2);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kRearLeftDrivingCanId = 3;
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kRearRightDrivingCanId = 2;

        public static final int kFrontLeftTurningCanId = 5;
        public static final int kRearLeftTurningCanId = 7;
        public static final int kFrontRightTurningCanId = 8;
        public static final int kRearRightTurningCanId = 6;

        public static final double kTurnToleranceDeg = 0;
        public static final double kTurnRateToleranceDegPerS = 0;

        public static final boolean kGyroReversed = true;

        public static final double kRobotStartOffset = 180;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 60; // amps
        public static final int kTurningMotorCurrentLimit = 30; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3.895; // max capable = 4.6
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // max capable = 7.4
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1.05;
        public static final double kPYController = 1.05;
        public static final double kPThetaController = 0.95; // needs to be separate from heading control

        public static final double kPHeadingController = 0.02; // for heading control NOT PATHING
        public static final double kDHeadingController = 0.0025;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ArmConstants{
        public static final int kArm = 9;

        public static final double kArmEncoderMultiplier = Math.toRadians(360.0/2048.0);

        public static final double kArmP = 8.1409; //11.873;
        //8.1409
        public static final double kArmI = 0;
        public static final double kArmD = 0.27; //1.3071;
        //6.5677

        public static final double kArmFeedS = 0.011333; //0.15323;
        public static final double kArmFeedG = 0.25802; //0.34485;
        public static final double kArmFeedV = 5.5397; //5.0245;
        public static final double kArmFeedA = 0.83731; //0.23018;  }

        public static final double kArmInitialPosition = Math.toRadians(-84.75);
        public static final double kArmMaxVelocitySecond = Math.toRadians(118*(4.0/5.0)); //118
        public static final double kArmMaxAccelRadPerSecond2 = Math.toRadians(215*(0.7)); //215
        public static final double kArmFloorAngle = Math.toRadians(-70);
        public static final double kArmLevel1Angle = Math.toRadians(-55);
        public static final double kArmLevel2Angle = Math.toRadians(-16);
        public static final double kArmLevel3Angle = Math.toRadians(-1);
        public static final double kArmHumanPlayerStationAngle = Math.toRadians(-17.5);

        public static final int kSourceChannelArmEncoder1 = 0;
        public static final int kSourceChannelArmEncoder2 = 1;
        public static final int kSourceChannelArmSwitch = 6;
    }

    public static final class WristConstants{
        public static final int kWrist = 10;

        public static final int kSourceChannelWristEncoder1 = 2;
        public static final int kSourceChannelWristEncoder2 = 3;
        public static final int kSourceChannelWristSwitch = 7;

        public static final double kWristEncoderMultiplier = Math.toRadians((360.0/2048.0)*(16.0/44.0));

        //p = 7.3191
        //d = 5.1308
        //8.614
        //1.0482
        public static final double kWristP = 30;
        public static final double kWristI = 0;
        public static final double kWristD = 0;

        /*  public static final double kWristFeedS = 2.2473;
        public static final double kWristFeedG = 2.236;
        public static final double kWristFeedV = 3.3042;
        public static final double kWristFeedA = 0.34277;
    */

        public static final double kWristFeedS = -1.3874;
        public static final double kWristFeedG = 3.5836;
        public static final double kWristFeedV = 4.8122;
        public static final double kWristFeedA = 0.33904; 

        public static final double kWristMaxVelocityPerSecond = Math.toRadians(90);
        public static final double kWristMaxAccelerationPerSecond2 = Math.toRadians(78);
        public static final double kWristStowedAngle = 2.8587;
        public static final double kWristFloorAngle = 0.0;//Math.toRadians(60);
        public static final double kWristLevel1Angle = Math.toRadians(55);
        public static final double kWristLevel2Angle = Math.toRadians(16);
        public static final double kWristLevel3Angle = Math.toRadians(1);
        public static final double kWristHumanPlayerStationAngle = 0;
    }

    public static final class ManipulatorConstants{
        public static final int kClawPCMOut = 3;
        public static final int kClawPCMIn = 0;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}