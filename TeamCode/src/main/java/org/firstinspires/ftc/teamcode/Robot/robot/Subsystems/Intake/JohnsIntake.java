package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

@Config
public class JohnsIntake extends Subsystem {

    CRServo rightintake,leftintake;
    Servo gripper,rightarm,leftarm;

    public static double outback = .78, down = .14;

    ColorSensor colorsensor;

    @Override
    public void initAuto(HardwareMap hwMap) {
        rightintake = hwMap.get(CRServo.class,"rightintake");
        leftintake = hwMap.get(CRServo.class,"leftintake");
        gripper = hwMap.get(Servo.class,"gripper");
        rightarm = hwMap.get(Servo.class,"rightarm");
        leftarm = hwMap.get(Servo.class,"leftarm");

        leftintake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightarm.setDirection(Servo.Direction.REVERSE);

        colorsensor = hwMap.get(ColorSensor.class,"colorsensor");
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {

    }

    public void setIntake(IntakeStates intakeStates){
        switch (intakeStates){
            case intake:
                rightintake.setPower(1);
                leftintake.setPower(1);
                break;
            case outtake:
                rightintake.setPower(-1);
                leftintake.setPower(-1);
                break;
            case stop:
                rightintake.setPower(0);
                leftintake.setPower(0);
                break;
        }
    }

    public void setGripper(GripperStates gripperStates) {
        switch (gripperStates){
            case unclamp:
                gripper.setPosition(.1);
                break;
            case clamp:
                gripper.setPosition(.73);
                break;
        }
    }

    public void setArmStates(ArmStates armStates){
        switch (armStates){
            case forward:
                rightarm.setPosition(down); // 121
                leftarm.setPosition(down);
                break;
            case parallel:
                rightarm.setPosition(.27);
                leftarm.setPosition(.27); //143
                break;
            case outback:
                rightarm.setPosition(outback);//227
                leftarm.setPosition(outback);
                break;
        }
    }

    public enum IntakeStates{
        intake,
        outtake,
        stop
    }
    public enum GripperStates{
        clamp,
        unclamp
    }
    public enum ArmStates{
        outback,
        parallel,
        forward
    }
}
