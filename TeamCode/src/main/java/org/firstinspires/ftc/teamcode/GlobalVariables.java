package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class GlobalVariables{

     public static Pose2d m_autonomousEndPose = new Pose2d(0, 0, Math.toRadians(-180));
     public static int closestTagID = -1;
     public static int randomization = 1;
     public static boolean firstCycle = true;

     public enum RobotState{
          Home
          ,ReadyToIntake
          ,Stow
          ,ReadyToDeploy
          ,Deploying
          ,Climb
          ,ReadyToLaunch
          ,TransitioningToIntake
          ,TransitioningToHome
          ,TransitioningToDeploy
          ,TransitioningToStow
     }

}
