package org.firstinspires.ftc.teamcode.drive;


import static com.acmerobotics.roadrunner.util.Angle.norm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.first.math.controller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.first.math.trajectory.TrapezoidProfile;
@Config
public class SwerveModule {
    private DcMotorEx motor=null;
    private CRServo servo=null;
    private AnalogInput m_turningEncoder=null;
    private Telemetry tel=null;
    private double m_zeroEncVolts, m_maxEncVolts;
    public int moduleNumber;
    public Double servoPower=0.0;
    private SwerveModuleState modState;
    private boolean powerflipping=false;
    private double lastangle=0, current=0, delt=0, delt1=0, delt2=0, angTarget=0;
    private boolean angChanged=false;
    private double pie=Math.PI, Twopie=2*Math.PI;

    // private Telemetry  tel;

    private PIDFController turnController;

    public SwerveModule(HardwareMap hwMap, DcMotorEx mot, CRServo serv, AnalogInput sens, double maxEncVolts, double zeroEncVolts, int modnum){

        this.motor=mot;
        this.servo=serv;
        this.m_zeroEncVolts=zeroEncVolts;
        this.moduleNumber=modnum;
        this.m_turningEncoder= sens;
        this.m_maxEncVolts=maxEncVolts;

        turnController=new PIDFController(new PIDCoefficients(DriveConstants.kPModuleTurningController,0,DriveConstants.kDModuleTurningController));
        turnController.setOutputBounds(-1,1);
        turnController.setInputBounds(0, 2*Math.PI);
    };

    public double getWheelAngle(){
        // radian
        double wheelAngleVoltage = ((m_turningEncoder.getVoltage()-m_zeroEncVolts));
        if (wheelAngleVoltage<0) wheelAngleVoltage+=m_maxEncVolts;

        return ((wheelAngleVoltage / m_maxEncVolts) * 2 * Math.PI);
    }

    public double getWheelAngleVoltage() {
        return m_turningEncoder.getVoltage();
    }
    public double  getWheelVelocity(){
        return motor.getVelocity();
    }

    public double getWheelPosition(){
        return motor.getCurrentPosition();
    }

    public void setWheelPower(double power){
        if(Math.abs(power)<0.03) power=0;
        motor.setPower(power);
    }

//    public SwerveModuleState getState(){
//        modState.modSpeed=getWheelVelocity();
//        modState.modAng=getWheelAngle();
//        return modState;
//    }

    public void setModuleState(double power, double angle){

        angle=norm(angle);
        double delta=angle-getWheelAngle();
        if(Math.abs(delta)>Math.PI){
            delta=delta<Math.PI ? delta+Twopie : delta-Twopie;
        }

        if(Math.abs(delta)>Math.PI/2){
            powerflipping=true;
            angle=norm(angle+Math.PI);
            power=-power;
        } else{
            powerflipping=false;
        }

        setWheelPower(power);
        setWheelAngle(power,angle);
    }

    public void setWheelAngle(double power, double pos){

        angChanged= lastangle != pos ?  true : false;
        current = getWheelAngle();

        if(Math.abs(power)>0.03 && angChanged) angTarget=pos;

        turnController.setTargetPosition(angTarget);
        double turnPower = turnController.update(current);
        turnPower += (Math.signum(turnPower) * DriveConstants.ksModuleTurningController);

        servo.setPower(Range.clip(turnPower, -1, 1));

        //  tel.addData("angle ", pos);
        //  tel.addData("turn power ",turnController.update(pos));
        //  tel.update();

        lastangle=angTarget;

    }

    public void setWheelAngleOnly(double pos){

        angChanged= angTarget != pos ?  true : false;
        current = getWheelAngle();
        if(angChanged) {
            angTarget = pos;
        /*
            delt=pos-current;
            delt1=delt % (2*Math.PI);
            delt2=delt % (-2*Math.PI);
            delt1=Math.abs(delt1)<Math.abs(delt2) ? delt1 : delt2;
            current=pos-delt1;

         */

            turnController.setTargetPosition(angTarget);
        }

        servo.setPower(Range.clip(turnController.update(current), -1.0, 1.0));
    }

}














