package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

public class Arm extends Subsystem {

    // Define the counts per revolution for your encoder
    public static final double COUNTS_PER_REVOLUTION = 751.8; // Adjust as needed

    // Define the gear ratio (output gear teeth / input gear teeth)
    public static final int GEAR_RATIO = 3, minarmconstraint = 0, maxarmconstraint = 600; // Adjust as needed

    public static int ref = 0, lowbasket = 300, highbasket = 500;
    DcMotor arm;

    public static PIDCoefficients pid = new PIDCoefficients(0,0,0);

    BasicPID controller = new BasicPID(pid);

    @Override
    public void initAuto(HardwareMap hwMap) {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {
        arm.setPower(0);

    }
    public void resetArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getArmPos(){
        return arm.getCurrentPosition();
    }
    // Method to convert encoder ticks to degrees

//    TODO: Check to make sure this method works.
    public static double ticksToDegrees(int ticks) {
        // Calculate the degrees per tick
        double degreesPerTick = 360.0 / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
        // Convert ticks to degrees
        return ticks * degreesPerTick;
    }
    public void armPID(int ref){
        if (ref < maxarmconstraint && ref > minarmconstraint) {
            arm.setPower(controller.calculate(ticksToDegrees(ref), getArmPos()));
        } else {
            arm.setPower(0);
        }
    }

    public void armPIDTicks(int ref){
        arm.setPower(controller.calculate(ref,getArmPos()));
    }

    public void setArmStates(ArmStates armStates){
        switch (armStates){
            case ScoreHighBasket:
                armPID(highbasket);
                break;
            case ScoreLowBasket:
                armPID(lowbasket);
                break;
            case Down:
                armPID(0);
                break;
        }
    }
    public double getArmError(){
        return ref - getArmPos();
    }
    public enum ArmStates{
        ScoreHighBasket,
        ScoreLowBasket,
        Down
    }
}
