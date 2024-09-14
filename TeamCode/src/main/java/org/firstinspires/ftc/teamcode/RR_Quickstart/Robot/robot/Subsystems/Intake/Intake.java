package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;

@Config
public class Intake extends Subsystem {

    public DcMotorEx intake;
    LowPassFilter lowPassFilter = new LowPassFilter(.2);
// motor and servo
    Servo intakeservo;
//    public static double counter12 = 1;
//    IntakeHeightStates heightStates = IntakeHeightStates.OnePixel;
    public static double intakeHigh = .4, intakeMaxSpeed = 1;  // default height and speed
    @Override
    public void initAuto(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake"); // hardware map motor and servo
        intakeservo = hwMap.get(Servo.class,"intakeservo");
//        counter12 = 1;

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set mode
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setIntakeHeight(IntakeHeightStates.Hang);  // set intake height for hanging
    }


    @Override
    public void initTeleop(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake"); // hardware map motor and servo
        intakeservo = hwMap.get(Servo.class,"intakeservo");
//        counter12 = 1;

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set mode
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setIntakeHeight(IntakeHeightStates.OnePixel);  // set intake height for hanging
    }

    @Override
    public void periodicAuto() {
//        Dashboard.addData("Intake Current",getCurrent()); // telmetry
//        Dashboard.addData("Filtered Curret", getFilteredCurrent());
    }

    @Override
    public void shutdown() {
        //TODO test to see if the schedular actually needs this to stop running the intake.
      //  setIntakePower(IntakeStates.Stop);
    }

    public double getCurrent(){
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public double getFilteredCurrent(){
        return lowPassFilter.estimate(intake.getCurrent(CurrentUnit.AMPS));
    }

    public void setIntakePower(IntakeStates intakeStates){
        // states for intake power
        switch (intakeStates){
            case IntakeAuto:
                intake.setPower(1);
                break;
            case Run:
                intake.setPower(intakeMaxSpeed); // fast
                break;
            case Run_Slow:
                intake.setPower(0.7); // slow
                break;
            case Reverse:
                intake.setPower(-1);  // reverse fast
                break;
            case ReverseSlow:
                intake.setPower(-.7);  // reverse slow
                break;
            case ReverseREALLYSlow:
                intake.setPower(-.6);
                break;
            case Stop: // slow
                intake.setPower(0);
                break;
        }
    }

    public void setIntakeHeight(IntakeHeightStates intakeHeightStates){
        // states for intake height
        switch (intakeHeightStates){
            case High:
                intakeservo.setPosition(intakeHigh);  // high intake
                break;
            case FivePixels:
                intakeservo.setPosition(.12);  // five pixels on stack
                break;
            case FourPixels:
                intakeservo.setPosition(.145);  // four pixels on stack
                break;
            case ThreePixels:
                intakeservo.setPosition(.17);  // three pixels on stack
                break;
            case TwoPixels:
                intakeservo.setPosition(.195);  // two pixels on stack
                break;
            case OnePixel:
                intakeservo.setPosition(.225);  // solo pixel
                break;
            case Hang:
                intakeservo.setPosition(.05);  // when hanging we don't want it to hit anything so smallest position possible
                break;
        }
    }

//    public void setIntakeHeightEasy(Input input){
//        if (input.isCrossPressed() && counter12 == 1){
//         heightStates = IntakeHeightStates.FivePixels;
//         counter12 = 2;
//        } else if (input.isCrossPressed() && counter12 ==2) {
//            heightStates = IntakeHeightStates.FourPixels;
//            counter12 = 3;
//        } else if (input.isCrossPressed() && counter12 == 3) {
//            heightStates = IntakeHeightStates.ThreePixels;
//            counter12 = 4;
//        } else if (input.isCrossPressed() && counter12 ==4) {
//            heightStates = IntakeHeightStates.TwoPixels;
//            counter12 = 5;
//        } else if (input.isCrossPressed()&& counter12 == 5) {
//            heightStates = IntakeHeightStates.OnePixel;
//            counter12 = 1;
//        }
//    }

    public enum IntakeStates{
        // create states
        IntakeAuto,
        Run,
        Run_Slow,
        Reverse,
        ReverseSlow,
        ReverseREALLYSlow,
        Stop
    }

    public enum IntakeHeightStates{
        // create states
        High,
        FivePixels,
        FourPixels,
        ThreePixels,
        TwoPixels,
        OnePixel,
        Hang
    }
}
