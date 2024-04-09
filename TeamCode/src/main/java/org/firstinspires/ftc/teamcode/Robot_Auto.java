package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.first.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;

public abstract class Robot_Auto extends LinearOpMode {

    public RobotContainer_RR m_robot;
    public int m_Analysis;

    private boolean m_redAlliance;

    private Pose2d m_startingPose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
    SequentialCommandGroup tasks;

    private ElapsedTime m_runTime = new ElapsedTime();

    public Robot_Auto(boolean detectRedAlliance) {
        this.m_redAlliance = detectRedAlliance;
    }

    public void initialize() {
        telemetry.clearAll();
        telemetry.addData("init complete", "BaseRobot");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeSubsystems();

        prebuildTasks();
        // waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            m_robot.run(); // run the scheduler
            telemetry.update();
        }

        m_Analysis = 1; // override for testing

        buildTasks(m_Analysis);

        m_runTime.reset();

        while (!isStopRequested() && opModeIsActive()) {
            m_robot.run(); // run the scheduler

//            m_robot.drivetrain.update();
//            Pose2d poseEstimate = m_robot.drivetrain.getPoseEstimate();
//            telemetry.addData("Position:","x[%3.2f] y[%3.2f] heading(%3.2f)", poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading());
            telemetry.update();
        }

        //
//        GlobalVariables.m_autonomousEndPose = m_robot.drivetrain.getPoseEstimate();
        endOfOpMode();
        m_robot.reset();
    }

    public void endOfOpMode() {

    }

    public void initializeSubsystems() {
        m_robot = new RobotContainer_RR(this);
        m_robot.drivetrain.setDefaultCommand(
                new RunCommand(()-> m_robot.drivetrain.drive(0,0,0),m_robot.drivetrain)
        );
    }

    public void setStartingPose(Pose2d p_pose) {
        m_startingPose = p_pose;
//        m_robot.drivetrain.resetOdometry(m_startingPose);
//        m_robot.drivetrain.zeroHeading();
    }

    public Pose2d getStartingPose() {
        return m_startingPose;
    }

    public abstract SequentialCommandGroup buildTasks(int m_Analysis);
    public abstract void prebuildTasks();

}