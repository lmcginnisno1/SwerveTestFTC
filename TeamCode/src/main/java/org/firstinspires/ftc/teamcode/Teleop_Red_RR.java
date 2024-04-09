package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.ftclib.command.button.Button;
import org.firstinspires.ftc.teamcode.ftclib.command.button.GamepadButton;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;
@TeleOp(name = "Teleop Red RR", group ="teleop red")
public class Teleop_Red_RR extends LinearOpMode {

     public RobotContainer_RR m_robot;
     //     private GamepadEx m_driverOp;
//     private GamepadEx m_toolOp;
     private boolean m_setFieldCentric = false;

     private static ElapsedTime m_runTime = new ElapsedTime();
     private ElapsedTime m_releaseTimeout = new ElapsedTime();

     public void initialize() {
          telemetry.clearAll();
          telemetry.addData("init complete", true);

          m_runTime.reset();
     }

     @Override
     public void runOpMode() throws InterruptedException {
          initializeSubsystems();

          // waitForStart();
          while (!opModeIsActive() && !isStopRequested()) {

               telemetry.update();
          }

          m_runTime.reset();
          while (!isStopRequested() && opModeIsActive()) {
               m_robot.run(); // run the scheduler

//               m_robot.drivetrain.update();
//               Pose2d poseEstimate = m_robot.drivetrain.getPoseEstimate();
//               telemetry.addData("ODM:","x[%3.2f] y[%3.2f] heading(%3.2f)", poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));//

               telemetry.update();
          }

          //
          endOfOpMode();
          m_robot.reset();
     }

     public void endOfOpMode() {

     }

     public void initializeSubsystems() {
          m_robot = new RobotContainer_RR(this);
//          m_driverOp = new GamepadEx(gamepad1);
//          m_toolOp = new GamepadEx(gamepad2);

          setSide();

          //drivetrain initialization
//          m_robot.drivetrain.setPoseEstimate(GlobalVariables.m_autonomousEndPose);

          // WPI
          m_robot.drivetrain.setDefaultCommand(
                  new RunCommand(
                          ()-> m_robot.drivetrain.drive(
                                  gamepad1.left_stick_y,
                                  gamepad1.left_stick_x,
                                  gamepad1.right_stick_x
                          ),
                          m_robot.drivetrain
                  )
          );

//          m_robot.drivetrain.zeroHeading();
//          m_robot.drivetrain.resetEncoders();
//          m_robot.drivetrain.resetOdometry(new Pose2d());

          configureButtonBindings();
     }

     public void configureButtonBindings() {

     }

     public void setSide() {
          m_robot.setRedSide();
     }

     public Button AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
          return (new GamepadButton(gamepad, button)).whenPressed(command);
     }

     public Button AddButtonCommandNoInt(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
          return (new GamepadButton(gamepad, button)).whenPressed(command, false);
     }
}