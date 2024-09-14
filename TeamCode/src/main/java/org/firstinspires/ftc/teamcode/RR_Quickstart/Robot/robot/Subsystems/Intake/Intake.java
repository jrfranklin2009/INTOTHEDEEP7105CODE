package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

import java.security.PublicKey;

@Config
public class Intake extends Subsystem {

    public CRServo intake;

    public Servo wrist;



    @Override
    public void initAuto(HardwareMap hwMap) {
        wrist = hwMap.get(Servo.class, "wrist");
        intake = hwMap.get(CRServo.class,"intake");
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        wrist = hwMap.get(Servo.class, "wrist");
        intake = hwMap.get(CRServo.class,"intake");

    }
    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {

    }

    public void setIntakePower(IntakePower intakePower){
        switch(intakePower){
            case Intake:
                intake.setPower(1);
            case Outtake:
                intake.setPower(-1);
            case Stop:
                intake.setPower(0);
        }
    }

    public void setWrist(Wrist wriststates){
        switch(wriststates){
            case Sample:
                wrist.setPosition(0.0);
            case PlacingSpecimin:
                wrist.setPosition(0.25);
        }

    }



    public enum Wrist{
        PlacingSpecimin,
        Sample
    }

    public enum IntakePower{
        Intake,
        Outtake,
        Stop
    }
}
