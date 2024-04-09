package org.firstinspires.ftc.teamcode.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.first.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.first.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveDriveOdometry;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveModulePosition;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.first.math.util.Units;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class SwerveDriveSubsystem_WPI extends SubsystemBase {
    // Robot swerve modules
    OpMode m_opMode;
    private SwerveModule m_frontLeft, m_rearLeft, m_frontRight, m_rearRight;
    private SwerveModuleState[] m_swerveModuleStates;
    private IMU m_imu;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    // The gyro sensor
//    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private VoltageSensor batteryVoltageSensor;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry=null;
    private List<SwerveModule> m_modules;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    private TrajectoryFollower follower;
    private TrajectorySequenceRunner trajectorySequenceRunner;


    /** Creates a new DriveSubsystem. */
    public SwerveDriveSubsystem_WPI(OpMode p_opMode) {
        m_opMode = p_opMode;
        HardwareMap hardwareMap = m_opMode.hardwareMap;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        m_frontLeft = new SwerveModule(hardwareMap,DriveConstants.kFrontLeftDriveMotorName
                ,DriveConstants.kFrontLeftTurningMotorName
                ,DriveConstants.kFrontLeftTurningEncoderName
                ,DriveConstants.kFrontLeftTurningEncoderZeroVoltage,DriveConstants.kFrontLeftTurningEncoderMaxVoltage
                ,DriveConstants.kFrontLeftTurningkS
                ,DriveConstants.kFrontLeftDriveEncoderReversed,DriveConstants.kFrontLeftTurningEncoderReversed);
        m_rearLeft = new SwerveModule(hardwareMap,DriveConstants.kRearLeftDriveMotorName
                ,DriveConstants.kRearLeftTurningMotorName
                ,DriveConstants.kRearLeftTurningEncoderName
                ,DriveConstants.kRearLeftTurningEncoderZeroVoltage,DriveConstants.kRearLeftTurningEncoderMaxVoltage
                ,DriveConstants.kRearLeftTurningkS
                , DriveConstants.kRearLeftDriveEncoderReversed,DriveConstants.kRearLeftTurningEncoderReversed);
        m_frontRight = new SwerveModule(hardwareMap,DriveConstants.kFrontRightDriveMotorName
                ,DriveConstants.kFrontRightTurningMotorName
                ,DriveConstants.kFrontRightTurningEncoderName
                ,DriveConstants.kFrontRightTurningEncoderZeroVoltage,DriveConstants.kFrontRightTurningEncoderMaxVoltage
                ,DriveConstants.kFrontRightTurningkS
                , DriveConstants.kFrontRightDriveEncoderReversed,DriveConstants.kFrontRightTurningEncoderReversed);
        m_rearRight = new SwerveModule(hardwareMap,DriveConstants.kRearRightDriveMotorName
                ,DriveConstants.kRearRightTurningMotorName
                ,DriveConstants.kRearRightTurningEncoderName
                ,DriveConstants.kRearRightTurningEncoderZeroVoltage,DriveConstants.kRearRightTurningEncoderMaxVoltage
                ,DriveConstants.kRearRightTurningkS
                , DriveConstants.kRearRightDriveEncoderReversed,DriveConstants.kRearRightTurningEncoderReversed);

        m_modules = Arrays.asList(m_frontLeft, m_rearLeft, m_rearRight, m_frontRight);

        // TODO: adjust the names of the following hardware devices to match your configuration

//        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
//                AHRS.DeviceDataType.kProcessedData,
//                NAVX_DEVICE_UPDATE_RATE_HZ);

        m_imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        m_imu.initialize(parameters);

        m_odometry =
                new SwerveDriveOdometry(
                        DriveConstants.kDriveKinematics,
                        getGyroRotation2d(),
                        new SwerveModulePosition[] {
                                m_frontLeft.getPosition(),
                                m_frontRight.getPosition(),
                                m_rearLeft.getPosition(),
                                m_rearRight.getPosition()
                        });

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                getGyroRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        if (m_swerveModuleStates != null) setModuleStates(m_swerveModuleStates);
        telemetry();
    }

    public void telemetry() {
        m_opMode.telemetry.addData("gyro heading:","% 4.1f",getHeading());
        Pose2d pose = getPose();
        m_opMode.telemetry.addData("Robot Pose:","heading=% 4.1f x=% 4.1f y=% 4.1f ",pose.getRotation().getDegrees(), Units.metersToInches(pose.getX()),Units.metersToInches(pose.getY()));

        m_opMode.telemetry.addData("SwerveAng","lf=% 3.0f lr=% 3.0f rf=% 3.0f rr=% 3.0f"
                ,Math.toDegrees(m_frontLeft.getWheelAngleRadian())
                ,Math.toDegrees(m_rearLeft.getWheelAngleRadian())
                ,Math.toDegrees(m_frontRight.getWheelAngleRadian())
                ,Math.toDegrees(m_rearRight.getWheelAngleRadian())
        );
        m_opMode.telemetry.addData("DesiredAng","lf=% 3.0f lr=% 3.0f rf=% 3.0f rr=% 3.0f"
                ,Math.toDegrees(m_frontLeft.getDesiredAngle())
                ,Math.toDegrees(m_rearLeft.getDesiredAngle())
                ,Math.toDegrees(m_frontRight.getDesiredAngle())
                ,Math.toDegrees(m_rearRight.getDesiredAngle())
        );
        m_opMode.telemetry.addData("WheelTicks","lf=% 5.0f lr=% 5.0f rf=% 5.0f rr=% 5.0f"
                ,(m_frontLeft.getWheelPosition())
                ,(m_rearLeft.getWheelPosition())
                ,(m_frontRight.getWheelPosition())
                ,(m_rearRight.getWheelPosition())
        );
        m_opMode.telemetry.addData("SwerveVolt","lf=% 6.3f lr=% 6.3f rf=% 6.3f rr=% 6.3f"
                ,(m_frontLeft.getWheelAngleVoltage())
                ,(m_rearLeft.getWheelAngleVoltage())
                ,(m_frontRight.getWheelAngleVoltage())
                ,(m_rearRight.getWheelAngleVoltage())
        );
    }
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                getGyroRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        m_swerveModuleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(
                        ChassisSpeeds.discretize(
                                fieldRelative
                                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, getGyroRotation2d())
                                        : new ChassisSpeeds(xSpeed, ySpeed, rot),
                                DriveConstants.kDrivePeriod));
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        drive(xSpeed, ySpeed,rot,false);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_imu.resetYaw();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return getImuHeadingVelocity() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    public Double getImuHeadingVelocity() {
        return (double) m_imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }
    public Rotation2d getGyroRotation2d() {
        return new Rotation2d( getRawExternalHeading() );
    }

    protected double getRawExternalHeading() {
        return m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        m_frontLeft.setPIDFCoefficients(runMode, coefficients);
        m_frontRight.setPIDFCoefficients(runMode, coefficients);
        m_rearLeft.setPIDFCoefficients(runMode, coefficients);
        m_rearRight.setPIDFCoefficients(runMode, coefficients);
    }

    public void setMode(DcMotor.RunMode mode) {
        m_frontLeft.setMode(mode);
        m_frontRight.setMode(mode);
        m_rearLeft.setMode(mode);
        m_rearRight.setMode(mode);
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (SwerveModule mod : m_modules) {
            double vel =  mod.getWheelSpeed();
            wheelVelocities.add(vel);
        }
        return wheelVelocities;
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseVelocity())
                        .turn(angle)
                        .build()
        );
    }

    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        Pose2d pose = getPose();
        return new com.acmerobotics.roadrunner.geometry.Pose2d(Units.metersToInches(pose.getX()),Units.metersToInches(pose.getY()),pose.getRotation().getRadians());
    }

    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),m_frontRight.getState(),m_rearLeft.getState(),m_rearRight.getState());
        // Getting individual speeds
//        double forward = chassisSpeeds.vxMetersPerSecond;
//        double sideways = chassisSpeeds.vyMetersPerSecond;
//        double angular = chassisSpeeds.omegaRadiansPerSecond;
        return new com.acmerobotics.roadrunner.geometry.Pose2d(chassisSpeeds.vxMetersPerSecond,chassisSpeeds.vyMetersPerSecond,chassisSpeeds.omegaRadiansPerSecond);
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }
    public TrajectoryBuilder trajectoryBuilder(com.acmerobotics.roadrunner.geometry.Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(com.acmerobotics.roadrunner.geometry.Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(com.acmerobotics.roadrunner.geometry.Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    public TrajectorySequenceBuilder trajectorySequenceBuilder(com.acmerobotics.roadrunner.geometry.Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }
    public void update() {
//        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDrive_Signal(signal);
    }

    public void setDrive_Signal(DriveSignal driveSignal) {
//        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }
    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}