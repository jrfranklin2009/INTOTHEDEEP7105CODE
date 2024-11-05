package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;

@Config
public class VerticalSlides extends Subsystem {

    DcMotor rightslide,leftslide;

    public static PIDCoefficients coefficients = new PIDCoefficients(0,0,0);

    public static double ref;

    BasicPID controller = new BasicPID(coefficients);

    @Override
    public void initAuto(HardwareMap hwMap) {
        ref = 0;
        rightslide = hwMap.get(DcMotor.class,"rightslide");
        leftslide = hwMap.get(DcMotor.class,"leftslide");

        rightslide.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {

    }

    public void pidController(){
        leftslide.setPower(controller.calculate(ref,getSlidesPos()));
        rightslide.setPower(controller.calculate(ref,getSlidesPos()));
    }

    public double getSlidesError(){
        return ref - getSlidesPos();
    }

    public void updatePos(Input input){
        if (input.isRightBumperPressed() && getSlidesPos() <= 800){
            ref = ref + 200;
        } else if (input.isLeftBumperPressed() && getSlidesPos() >= 200){
            ref = ref - 200;
        }
    }

    public double getSlidesPos(){
        return leftslide.getCurrentPosition();
    }

    public enum VerticalSlidesStates{
        Up,
        Down
    }

}
