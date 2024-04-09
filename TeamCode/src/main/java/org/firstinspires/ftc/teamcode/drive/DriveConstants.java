package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */

    public static final boolean useOdoPods=false;   //odometry pods
    public static  boolean RUN_USING_ENCODER = true;    //false when using odometry pods

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 348.92;
    public static final double MAX_RPM = 480;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    //public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30.0, 0.0, 0.0,17.0);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 34.5/25.4; // in  (roughly 3 in wheel)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 11.375; // in
    public static double WHEEL_BASE = 8.875; //in
    public static double DIAGONAL=Math.sqrt(TRACK_WIDTH*TRACK_WIDTH+WHEEL_BASE*WHEEL_BASE);

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0130;    //   500rpm at 12Volt   78 iches per second
    public static double kA = 0.003;
    public static double kStatic = 0.00;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 50;
    public static double MAX_ANG_VEL = 3.84;
    public static double MAX_ANG_ACCEL = 3.84;

    public static double MEDHIGH_VEL=45.0, BASE_VEL=30.0, HIGH_VEL=60.2, MED_VEL=40.0, LOW_VEL=20.0, CREEPER_VEL=15.0, SUPERCREEPER_VEL=10;     // was 3
    public static double MEDHIGH_ACL=45.0, BASE_ACL=30.0, HIGH_ACL=60.2, MED_ACL=40.0, LOW_ACL=20.0, CREEPER_ACL=15.0, SUPERCREEPER_ACL=10;   //was 5
    public static double MEDHIGHANG_VEL=Math.PI*2, BASEANG_VEL=Math.PI, HIGHANG_VEL=2*Math.PI, MEDANG_VEL=Math.PI, LOWANG_VEL=Math.PI/2, CREEPERANG_VEL=Math.PI/2, SUPERCREEPERANG_VEL=Math.PI/2;
    public static double MEDHIGHANG_ACL=Math.PI*2, BASEANG_ACL=Math.PI, HIGHANG_ACL=2*Math.PI, MEDANG_ACL=Math.PI, LOWANG_ACL=Math.PI/2, CREEPERANG_ACL=Math.PI/2, SUPERCREEPERANG_ACL=Math.PI/2;


    /*
     * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

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
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static double kPModuleTurningController = 0.5;
    public static double kDModuleTurningController = 0.005;
    public static double ksModuleTurningController = 0.01;
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    public static final int kEncoderCPR = 28 * (42/14) * (27/13) * (28/14) ; // = 348.92
    public static final double kWheelDiameterMeters = .06940 ;
    public static final double ksVolts = 0.0;
    public static final double kvVoltSecondsPerMeter = 0.0130;  //   500rpm at 12Volt   78 iches per second
    public static final double kaVoltSecondsSquaredPerMeter = 0.003;
    public static final double kMaxSpeedTicksPerSecond = 2600;
    public static final double kMaxSpeedMetersPerSecond = (kWheelDiameterMeters * Math.PI) * (kMaxSpeedTicksPerSecond / kEncoderCPR); // 1.629


}
