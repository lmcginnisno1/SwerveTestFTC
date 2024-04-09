package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.ftclib.command.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveSubsystem_WPI;


public class RobotContainer_WPI {
    public boolean m_red = true;
    public Robot m_robot = new Robot();
    public GlobalVariables m_variables;
    //    public SwerveDriveSubsystem_RR drivetrain;
    public SwerveDriveSubsystem_WPI drivetrain;
    public RobotContainer_WPI(OpMode p_opMode) {
//        SampleSwerveDrive drivebase = new SampleSwerveDrive(p_opMode.hardwareMap);
        drivetrain = new SwerveDriveSubsystem_WPI(p_opMode);
    };

    public boolean isRedSide() {
        return m_red;
    }
    public boolean isBlueSide() {
        return !m_red;
    }
    public double setSideMultiplier(double value) {
        return (m_red ? 1 : -1) * value;
    }

    public void setRedSide() {
        m_red = true;
    }

    public void setBlueSide() {
        m_red = false;
    }

    public double redSide(double value) {
        return m_red ? value : -value;
    }

    public double redSide(double value, double blue) {
        if (m_red)
            return value;
        else
            return blue;
    }

    public double blueSide(double value) {
        return m_red ? -value : value;
    }

    public double blueSide(double value, double red) {
        if (m_red)
            return red;
        else
            return value;
    }

    public void run() {
        m_robot.run();
    }

    public void reset() {
        m_robot.reset();
    }
    public void schedule(Command... commands) {
        m_robot.schedule(commands);
    }
}

