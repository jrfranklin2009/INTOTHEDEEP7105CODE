package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;

@Config
public class HorizontalSlides extends Subsystem {

    ServoImplEx leftservoslide, rightservoslide;

    public static double rightpos = .72, leftpos = .72,
    //TODO max and min constrains must be fixed
            maxconstraint = 21.5, minconstraint = 8.49;

    //get our analog input from the hardwareMap
    AnalogInput slideAnalog;

    //13 inches

    @Override
    public void initAuto(HardwareMap hwMap) {
        leftservoslide = hwMap.get(ServoImplEx.class,"leftservoslide");
        rightservoslide = hwMap.get(ServoImplEx.class,"rightservoslide");
        leftservoslide.setDirection(Servo.Direction.REVERSE);
        slideAnalog = hwMap.get(AnalogInput.class, "leftslideencoder");
    }

    @Override
    public void periodicAuto() {
        Dashboard.addData("horizontalslides",getSlidePos());
    }

    @Override
    public void shutdown() {

    }

    public double getSlidePos(){
        return slideAnalog.getVoltage() / 3.3 * 360;
    }

    public double getVoltage(){
        return slideAnalog.getVoltage();
    }

    public double getSlideError(double ref){
        return ref - getSlidePos();
    }

    public double ticksToInches(){
        return getSlidePos() / 16.84;
    }

    public void setHorizontalSlides(HorizontalSlideStates horizontalslidestates){
        switch (horizontalslidestates){
            case Fully_In:
                leftservoslide.setPosition(.15); //143
                rightservoslide.setPosition(.15);
                break;
            case Half_Out:
                leftservoslide.setPosition(.445);
                rightservoslide.setPosition(.445);
                break;
            case Fully_Out:
                leftservoslide.setPosition(leftpos); // 219
                rightservoslide.setPosition(rightpos);
                break;
            case Zero_Power:
                leftservoslide.setPwmDisable();
                rightservoslide.setPwmDisable();
                break;
        }
    }

    public enum HorizontalSlideStates {
        Fully_Out,
        Half_Out,
        Fully_In,
        Zero_Power
    }
}
