package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.jcip.annotations.GuardedBy;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;

@Config
public class VerticalSlides extends Subsystem {

    DcMotor rightslide,leftslide;

//    public static boolean rat = false;

//    LinearOpMode opMode;

//    private final Object slideLock = new Object();
//    @GuardedBy("slideThread")
//    private Thread slideThread  = new Thread(() -> {
//        while (!opMode.isStopRequested()) {
//            synchronized (slideLock) {
//                    slidepos = getSlidesPos();
//                pidController();
//                    imuAngle = AngleUnit.normalizeRadians(imu.getAngularOrientation().firstAngle);
//            }
//        }
//    });;
    public static double  lowchamber = 1210, lowbasket = 1630, highbasket =2330, ref, lastSlidePosition = 0;
    int counter = 0;
    public static boolean closeThread = false;

    public static PIDCoefficients coefficients = new PIDCoefficients(.008,0,.000000000002);

    BasicPID controller = new BasicPID(coefficients);

    public VerticalSlides(LinearOpMode opMode){
//        this.opMode = opMode;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        ref = 0;
        closeThread = false;
//        slidepos = 0;
        rightslide = hwMap.get(DcMotor.class,"rightslide");
        leftslide = hwMap.get(DcMotor.class,"leftslide");

        rightslide.setDirection(DcMotorSimple.Direction.REVERSE);

        resetSlides();

    }

    @Override
    public void periodicAuto() {
        Dashboard.addData("verticalslidepos",getSlidesPos());
        Dashboard.addData("reference",ref);
        Dashboard.addData("closethread",closeThread);
    }

    @Override
    public void shutdown() {

    }

    public void pidController(){
        leftslide.setPower(controller.calculate(ref,getSlidesPos()));
        rightslide.setPower(controller.calculate(ref,getSlidesPos()));
    }

    public void getAndSetPower(){
        double getPow = leftslide.getPower();
        leftslide.setPower(getPow);
        rightslide.setPower(getPow);
    }

    public void zeroPower(){
        leftslide.setPower(0);
        rightslide.setPower(0);
    }

    public double getChangeRate(){
        return getSlidesPos() - getLastSlidePos();
    }

    public double getLastSlidePos(){
        if (counter > 2){
            lastSlidePosition = getSlidesPos();
            counter = 1;
        } else {
            counter = counter+1;
        }
        return lastSlidePosition;
    }


    public double getSlidesError(){
        return ref - getSlidesPos();
    }

    public void startSLIDEThread() {
//        slideThread.start();
    }

    public void closeSLIDEThread(){
//        slideThread.interrupt();
        closeThread = true;
    }

    public boolean isThreadInterrupted(){
//        return slideThread.isInterrupted();
        return false;
    }

//    public void updatePos(Input input){
//        if (input.isRightBumperPressed() && getSlidesPos() <= 2400){
//            ref = ref + 300;
//        } else if (input.isLeftBumperPressed() && getSlidesPos() >= 150){
//            ref = ref - 300;
//        }
//    }

    public void updatePos(Input input){
        if (input.isRightBumperPressed()){
            ref = lowchamber;
        } else if (input.isLeftBumperPressed()){
            ref = 0;
        } else if (input.isRightBumperPressed() && ref == lowchamber){
            ref = lowbasket;
        } else if (input.isLeftBumperPressed() && ref == lowbasket){
            ref = lowchamber;
        }
    }

    public double getSlidesPos(){
        return leftslide.getCurrentPosition();
    }

    public void resetSlides(){
//        slidepos = 0;
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

public enum VerticalSlidesStates{
        Up,
        Down
    }

}
