package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.first.math.util.Units;

import java.util.List;

@Autonomous(name = "DriveVelocityPIDTuner", group = "Swerve Tuning")
public class Swerve_DriveVelocityPIDTuner extends LinearOpMode {
    public static double DISTANCE = 72; // in
    public RobotContainer_WPI m_robot;
    private ElapsedTime m_runTime = new ElapsedTime();
    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }
    Mode m_mode = Mode.TUNING_MODE;
    NanoClock m_clock = null;
    boolean m_movingForwards = true;
    MotionProfile m_activeProfile;
    double m_profileStart =0;
    Telemetry m_telemetry;
    double lastKp = MOTOR_VELO_PID.p;
    double lastKi = MOTOR_VELO_PID.i;
    double lastKd = MOTOR_VELO_PID.d;
    double lastKf = MOTOR_VELO_PID.f;
    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeSubsystems();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Mode mode = Mode.TUNING_MODE;
        double lastKp = MOTOR_VELO_PID.p;
        double lastKi = MOTOR_VELO_PID.i;
        double lastKd = MOTOR_VELO_PID.d;
        double lastKf = MOTOR_VELO_PID.f;

        m_robot.drivetrain.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        // waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            m_robot.run(); // run the scheduler
            telemetry.update();
        }

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();

        m_runTime.reset();
        while (!isStopRequested() && opModeIsActive()) {

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                        m_robot.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
//                    double targetPower = kV * motionState.getV();
                    m_robot.drivetrain.drive(Units.inchesToMeters(motionState.getV()),0,0);

                    List<Double> velocities = m_robot.drivetrain.getWheelVelocities();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    for (int i = 0; i < velocities.size(); i++) {
                        telemetry.addData("measuredVelocity" + i, Units.metersToInches(velocities.get(i)));
                        telemetry.addData(
                                "error" + i,
                                motionState.getV() - Units.metersToInches(velocities.get(i))
                        );
                    }
                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        m_robot.drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }
                    m_robot.drivetrain.drive(
                            gamepad1.left_stick_y * DriveConstants.kMaxSpeedMetersPerSecond,
                            gamepad1.left_stick_x * DriveConstants.kMaxSpeedMetersPerSecond,
                            gamepad1.right_stick_x * DriveConstants.kMaxModuleAngularSpeedRadiansPerSecond);
                    break;
            }

            if (lastKp != MOTOR_VELO_PID.p || lastKd != MOTOR_VELO_PID.d
                    || lastKi != MOTOR_VELO_PID.i || lastKf != MOTOR_VELO_PID.f) {
                m_robot.drivetrain.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            m_robot.run(); // run the scheduler
            telemetry.update();
        }
        //
        m_robot.reset();
    }

    public void initializeSubsystems() {
        m_robot = new RobotContainer_WPI(this);
        m_robot.drivetrain.setDefaultCommand(
                new RunCommand(()-> m_robot.drivetrain.drive(0,0,0),m_robot.drivetrain)
        );
    }
}
