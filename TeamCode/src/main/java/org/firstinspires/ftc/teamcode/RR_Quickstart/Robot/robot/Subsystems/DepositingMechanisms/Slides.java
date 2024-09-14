package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;


@Config
public class Slides extends Subsystem {
    public static double HighPos = 1450, MidPos = 550, MaxAccel = 30, MaxNegAccel = 65, MaxVel = 60;
    public static double kp = 0.008, ki = 0, kd = 0.00004, kg = .18, ref = 0;
    public static TouchSensor touchslides;
    public static double counter = 1;
    PIDCoefficients coefficients = new PIDCoefficients(kp,ki,kd);

    public DcMotorEx leftslide, rightslide;
    @Override
    public void initAuto(HardwareMap hwMap) {
        leftslide = hwMap.get(DcMotorEx.class,"leftslide");
        rightslide = hwMap.get(DcMotorEx.class,"rightslide");
        touchslides = hwMap.get(TouchSensor.class,"touchslides");
        leftslide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ref = 0;
        counter = 1;
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {
        leftslide.setPower(0);
        rightslide.setPower(0);
    }

    public double setTarget(double target){
        return ref = target;
    }

    public double getTarget(){
        return ref;
    }

    public void resetSlides() {
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getSlidePos(){
        return (leftslide.getCurrentPosition() + rightslide.getCurrentPosition()) / 2;
    }

    public double getLeftSlidePos(){
        return leftslide.getCurrentPosition();
    }

    public double getError(double target){
        return target - getSlidePos();
    }
}
