package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;

import java.util.List;

/**
 * A subsystem that uses the {@link SampleSwerveDrive} class.
 * This periodically calls {@link SampleSwerveDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class SwerveDriveSubsystem_RR extends SubsystemBase {
    private final SampleSwerveDrive drive;
    private boolean fieldCentric = false;
    private double m_headingAdjustRobotStartPosition = 0;

    OpMode m_opMode;

    public SwerveDriveSubsystem_RR(SampleSwerveDrive drive, OpMode p_opMode) {
        this.drive = drive;
        fieldCentric = false;

        m_opMode = p_opMode;
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
                fieldCentric ? -poseEstimate.getHeading() + m_headingAdjustRobotStartPosition : 0
        );

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightX
                )
        );
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectorySequence(TrajectorySequence sequence){
        drive.followTrajectorySequenceAsync(sequence);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    public void setMotorPowers(double p_power){
        drive.setMotorPowers(p_power, p_power, p_power, p_power);
    }

    public void setFieldCentric(boolean p_enabled ,double p_StartAngleDegree) {
        fieldCentric = p_enabled;
        m_headingAdjustRobotStartPosition = Math.toRadians(p_StartAngleDegree);
    }

    public void setFieldCentric(boolean p_enabled) {
        fieldCentric = p_enabled;
    }

    public void setFieldCentric(double p_StartAngleDegree) {
        m_headingAdjustRobotStartPosition = Math.toRadians(p_StartAngleDegree);
    }

    @Override
    public void periodic(){
        m_opMode.telemetry.addData(this.m_name, m_headingAdjustRobotStartPosition);
        List<Double> wheelVoltages = drive.getWheelVoltages();
        m_opMode.telemetry.addData("WheelVoltage","fL=%05.3f rL=%05.3f rR=%05.3f fR=%05.3f",wheelVoltages.get(0),wheelVoltages.get(1),wheelVoltages.get(2),wheelVoltages.get(3));
    }
}
