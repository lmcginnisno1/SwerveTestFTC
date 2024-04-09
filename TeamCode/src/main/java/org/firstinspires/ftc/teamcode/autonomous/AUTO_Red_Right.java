package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.first.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;

@Autonomous(name = "Red Right", group = "Auto Red", preselectTeleOp = "Teleop Red")
public class AUTO_Red_Right extends Robot_Auto {
    int m_Analysis;

    public AUTO_Red_Right() {
        super(true);
    }

    @Override
    public void prebuildTasks() {
        //run these tasks now
        setStartingPose(new Pose2d(0, 0, new Rotation2d().fromDegrees(0)));
    }

    @Override
    public SequentialCommandGroup buildTasks(int p_Analysis) {
        m_Analysis = p_Analysis;
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();
        completeTasks.addCommands(
                createTrajectory ()
        );
        m_robot.schedule(completeTasks);
        return completeTasks;
    }

    // Create config for trajectory
    public CommandBase createTrajectory () {
        return new CommandBase() {
        };
    };
}
