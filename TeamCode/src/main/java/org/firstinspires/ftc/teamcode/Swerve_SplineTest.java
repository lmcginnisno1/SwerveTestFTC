package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;

@Autonomous(name = "Spline Test", group = "Swerve Tuning")
public class Swerve_SplineTest extends LinearOpMode {
    public RobotContainer_WPI m_robot;
    private ElapsedTime m_runTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initializeSubsystems();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        // waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            m_robot.run(); // run the scheduler
            telemetry.update();
        }

        m_runTime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            Trajectory traj = m_robot.drivetrain.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(30, 30), 0)
                    .build();

            m_robot.drivetrain.followTrajectory(traj);

            sleep(2000);

            m_robot.drivetrain.followTrajectory(
                            m_robot.drivetrain.trajectoryBuilder(traj.end(), true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build()
            );
        }
    }

    public void initializeSubsystems() {
        m_robot = new RobotContainer_WPI(this);
        m_robot.drivetrain.setDefaultCommand(
                new RunCommand(()-> m_robot.drivetrain.drive(0,0,0),m_robot.drivetrain)
        );
    }
}
