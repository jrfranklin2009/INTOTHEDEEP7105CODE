package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

public class VerticalSlides extends Subsystem {

    DcMotor rightslide,leftslide;

    PIDCoefficients coefficients = new PIDCoefficients(0,0,0);

    BasicPID controller = new BasicPID(coefficients);

    @Override
    public void initAuto(HardwareMap hwMap) {
        rightslide = hwMap.get(DcMotor.class,"rightslide");
        leftslide = hwMap.get(DcMotor.class,"leftslide");

        leftslide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {

    }

    public void pidController(double ref){
        leftslide.setPower(controller.calculate(ref,getSlidesPos()));
        rightslide.setPower(controller.calculate(ref,getSlidesPos()));
    }

    public double getSlidesError(double ref){
        return ref - getSlidesPos();
    }

    public double getSlidesPos(){
        return leftslide.getCurrentPosition();
    }

    public enum VerticalSlidesStates{
        Up,
        Down
    }

}
