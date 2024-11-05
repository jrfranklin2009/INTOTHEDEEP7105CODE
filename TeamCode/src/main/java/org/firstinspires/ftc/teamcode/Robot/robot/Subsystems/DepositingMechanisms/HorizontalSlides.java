package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

public class HorizontalSlides extends Subsystem {

    Servo leftservoslide, rightservoslide;

    @Override
    public void initAuto(HardwareMap hwMap) {
        leftservoslide = hwMap.get(Servo.class,"leftservoslide");
        rightservoslide = hwMap.get(Servo.class,"rightservoslide");
        rightservoslide.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {

    }

    public void setHorizontalSlides(HorizontalSlideStates horizontalslidestates){
        switch (horizontalslidestates){
            case Fully_Out:
                leftservoslide.setPosition(.1);
                rightservoslide.setPosition(.1);
                break;
            case Fully_In:
                leftservoslide.setPosition(.3);
                rightservoslide.setPosition(.3);
                break;
        }
    }

    public enum HorizontalSlideStates {
        Fully_Out,
        Fully_In
    }
}
