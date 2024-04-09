package org.firstinspires.ftc.teamcode.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.first.math.controller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveModulePosition;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
    private final DcMotorEx m_driveMotor;
    private final CRServo m_turningMotor;
    private final AnalogInput m_turningEncoder;
    private VoltageSensor m_batteryVoltageSensor;
    private double m_zeroEncVolts,m_maxEncVolts;
    private double m_turningKs;
    private double m_drivePositionOffset=0;
    private boolean m_driveEncoderReversed = false, m_turningEncoderReversed = false;
    SwerveModuleState m_desiredState = new SwerveModuleState();
//    private static PIDController m_drivePIDController =
//            new PIDController(DriveConstants.kPModuleDriveController, 0, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private static ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    DriveConstants.kPModuleTurningController,
                    0.0,
                    DriveConstants.kDModuleTurningController,
                    new TrapezoidProfile.Constraints(
                            DriveConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                            DriveConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    /**
     * Constructs a SwerveModule.
     *
     * @param p_driveMotorName The channel of the drive motor.
     * @param p_turningMotorName The channel of the turning motor.
     * @param p_turningEncoderName The channels of the drive encoder.
     * @param driveEncoderReversed Whether the drive encoder is reversed.
     * @param turningEncoderReversed Whether the turning encoder is reversed.
     */
    public SwerveModule(HardwareMap hardwareMap,
                        String p_driveMotorName,
                        String p_turningMotorName,
                        String p_turningEncoderName,
                        double p_zeroEncVolts,
                        double p_m_maxEncVolts,
                        double p_turningKs,
                        boolean driveEncoderReversed,
                        boolean turningEncoderReversed) {

        m_batteryVoltageSensor =  hardwareMap.voltageSensor.iterator().next();
        m_driveMotor =  hardwareMap.get(DcMotorEx.class, p_driveMotorName);
        setDriveMotor();

        m_turningMotor = hardwareMap.get(CRServo.class, p_turningMotorName);;
        m_turningMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        m_turningEncoder = hardwareMap.get(AnalogInput.class, p_turningEncoderName);
        m_zeroEncVolts = p_zeroEncVolts;
        m_maxEncVolts = p_m_maxEncVolts;
        m_turningKs = p_turningKs;
        m_driveEncoderReversed = driveEncoderReversed;
        m_turningEncoderReversed = turningEncoderReversed;

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
//        m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
//        m_driveEncoder.setReverseDirection(driveEncoderReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
//        m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
//        m_turningEncoder.setReverseDirection(turningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(0, 2*Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getWheelSpeed(), new Rotation2d( getWheelAngleRadian()));
    }
    public double getWheelSpeed(){
        double velocityTickPerSecond = m_driveMotor.getVelocity();
        return  (velocityTickPerSecond * DriveConstants.kDriveEncoderDistancePerPulse);
    }
    public double getWheelAngleRadian(){
        // radian
        double wheelAngleVoltage = ((m_turningEncoder.getVoltage()-m_zeroEncVolts));
        if (wheelAngleVoltage<0) wheelAngleVoltage+=m_maxEncVolts;

        return ((wheelAngleVoltage / m_maxEncVolts) * 2 * Math.PI);
    }
    public double getWheelAngleVoltage() {
        return m_turningEncoder.getVoltage();
    }
    public double getWheelDistance() {
        return getWheelPosition() * DriveConstants.kDriveEncoderDistancePerPulse;
    }
    public double getWheelPosition() {
        return (m_driveMotor.getCurrentPosition() - m_drivePositionOffset) * (m_driveEncoderReversed ?-1:1);
    }
    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getWheelDistance(), new Rotation2d( getWheelAngleRadian() ));
    }
    public double getDesiredAngle() {
        return m_desiredState.angle.getRadians();
    }
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        m_desiredState= desiredState;
        var encoderRotation = new Rotation2d( getWheelAngleRadian() );

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        // Calculate the drive output from the drive PID controller.
//        final double driveOutput =
//                m_drivePIDController.calculate( getWheelSpeed() , state.speedMetersPerSecond);

        // Rev Control Hub controller velocity uses ticks per second
        double percentPower = (state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
        m_driveMotor.setPower(percentPower);

        // Calculate the turning motor output from the turning PID controller.
        double turnOutput =
                m_turningPIDController.calculate( getWheelAngleRadian(), state.angle.getRadians() );
        turnOutput += (Math.signum(turnOutput) * m_turningKs);

        m_turningMotor.setPower(Range.clip(turnOutput,-1,1));
    }
    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
//        m_driveEncoder.reset();
        m_drivePositionOffset = m_driveMotor.getCurrentPosition();
//        m_turningEncoder.reset();
    }
    public void setDriveMotor() {
        m_driveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients coefficients = DriveConstants.MOTOR_VELO_PID;
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / m_batteryVoltageSensor.getVoltage()
        );
        m_driveMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, compensatedCoefficients);
        setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,DriveConstants.MOTOR_VELO_PID);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / m_batteryVoltageSensor.getVoltage()
        );

        m_driveMotor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
    public void setMode(DcMotor.RunMode runMode) {
        m_driveMotor.setMode(runMode);
    }
}
