package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.first.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.first.math.util.Units;

public class DriveConstants {
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    /*
        In Java, everything is in meters
        the velocity of the wheel must be in meters per second
     */
    public static final String kFrontLeftDriveMotorName = "frontLeft";
    public static final String kRearLeftDriveMotorName = "rearLeft";
    public static final String kFrontRightDriveMotorName = "frontRight";
    public static final String kRearRightDriveMotorName = "rearRight";
    public static final String kFrontLeftTurningMotorName = "frontLeft_turn";
    public static final String kRearLeftTurningMotorName = "rearLeft_turn";
    public static final String kFrontRightTurningMotorName = "frontRight_turn";
    public static final String kRearRightTurningMotorName = "rearRight_turn";

    public static final String kFrontLeftTurningEncoderName = "frontLeft_enc";
    public static final String kRearLeftTurningEncoderName = "rearLeft_enc";
    public static final String kFrontRightTurningEncoderName ="frontRight_enc";
    public static final String kRearRightTurningEncoderName = "rearRight_enc";
    public static final double kFrontLeftTurningEncoderZeroVoltage  = 2.515;
    public static final double kRearLeftTurningEncoderZeroVoltage   = 3.046;
    public static final double kFrontRightTurningEncoderZeroVoltage = 2.237;
    public static final double kRearRightTurningEncoderZeroVoltage  =  .901;
    public static final double kFrontLeftTurningEncoderMaxVoltage = 3.285;
    public static final double kRearLeftTurningEncoderMaxVoltage = 3.25;
    public static final double kFrontRightTurningEncoderMaxVoltage = 3.272;
    public static final double kRearRightTurningEncoderMaxVoltage = 3.336;
    public static final double kFrontLeftTurningkS = 0.035;
    public static final double kRearLeftTurningkS = 0.035;
    public static final double kFrontRightTurningkS = 0.035;
    public static final double kRearRightTurningkS = 0.030;
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = .020;

    public static final double kTrackWidth = Units.inchesToMeters(11.375); //.288925;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(8.875); //.225425;
    // Distance between front and back wheels on robot

    /*
    The locations for the modules must be relative to the center of the robot.
    Positive x values represent moving toward the front of the robot
    whereas positive y values represent moving toward the left of the robot.
     */
    public static final Translation2d frontLeftWheelLocation = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d frontRightWheelLocation = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d rearLeftWheelLocation = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d rearRightWheelLocation = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    frontLeftWheelLocation,
                    frontRightWheelLocation,
                    rearLeftWheelLocation,
                    rearRightWheelLocation);

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final int kEncoderCPR = 28 * (42/14) * (27/13) * (28/14) ; // = 348.92
    public static final double kWheelDiameterMeters = .06900 ;
    public static final double ksVolts = 0.0;
    public static final double kvVoltSecondsPerMeter = 0.0130;  //   500rpm at 12Volt   78 iches per second
    public static final double kaVoltSecondsSquaredPerMeter = 0.003;
    public static final double kMaxSpeedTicksPerSecond = 2600;
    public static final double kMaxSpeedMetersPerSecond = (kWheelDiameterMeters * Math.PI) * (kMaxSpeedTicksPerSecond / kEncoderCPR); // 1.629
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30.0, 0.0, 0,17.0);
    public static final double kDriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderCPR;
    //    public static double kPModuleDriveController = 1.2;
    public static double kPModuleTurningController = 0.2;
    public static double kDModuleTurningController = 0.00;
}
