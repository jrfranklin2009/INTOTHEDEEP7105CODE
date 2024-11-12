package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

@Config
public class HorizontalSlides extends Subsystem {

    Servo leftservoslide, rightservoslide;

    public static double rightpos = .72, leftpos = .72;

    @Override
    public void initAuto(HardwareMap hwMap) {
        leftservoslide = hwMap.get(Servo.class,"leftservoslide");
        rightservoslide = hwMap.get(Servo.class,"rightservoslide");
        leftservoslide.setDirection(Servo.Direction.REVERSE);
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
                leftservoslide.setPosition(.17);
                rightservoslide.setPosition(.17);
                break;
            case Fully_In:
                leftservoslide.setPosition(leftpos);
                rightservoslide.setPosition(rightpos);
                break;
        }
    }

    public enum HorizontalSlideStates {
        Fully_Out,
        Fully_In
    }
}
