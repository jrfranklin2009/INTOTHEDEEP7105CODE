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

    public CRServo intake;  // intake wheel

    public Servo twist; // the turning thing that rotates

    public Servo wrist;  // flip-flop



    @Override
    public void initAuto(HardwareMap hwMap) {
        wrist = hwMap.get(Servo.class, "wrist");
        intake = hwMap.get(CRServo.class,"intake");
        twist = hwMap.get(Servo.class, "twist");
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        wrist = hwMap.get(Servo.class, "wrist");
        intake = hwMap.get(CRServo.class,"intake");
        twist = hwMap.get(Servo.class, "twist");

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
            case Rest:
                wrist.setPosition(0.0);
            case IntakeSample:
                wrist.setPosition(0.0);
            case OuttakeSample:
                wrist.setPosition(0.2);
            case PlacingSpecimin:
                wrist.setPosition(0.0);
        }

    }

    public void setTwist(Twist twiststates){
        switch(twiststates){
            case Rest:
                twist.setPosition(0.0);
            case PlacingSpecimin:
                twist.setPosition(0.25);
            case IntakeSample:
                twist.setPosition(0);
            case OuttakeSample:
                twist.setPosition(0);
        }
    }



    public enum Wrist{
        PlacingSpecimin,
        IntakeSample,
        OuttakeSample,
        Rest
    }

    public enum IntakePower{
        Intake,
        Outtake,
        Stop
    }

    public enum Twist{
        PlacingSpecimin,
        IntakeSample,
        OuttakeSample,
        Rest
    }
}
