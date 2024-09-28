package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

/** This is code for the arm
 * Currently is just kooky arm.
 */

public class Arm extends Subsystem {

    // Define the counts per revolution for your encoder
    public static final double COUNTS_PER_REVOLUTION = 751.8; // Adjust as needed

    // Define the gear ratio (output gear teeth / input gear teeth)
    public static final int GEAR_RATIO = 3, minarmconstraint = 0, maxarmconstraint = 600; // Adjust as needed

    public static int ref = 0, lowbasket = 300, highbasket = 500;
    DcMotor arm;  // define arm

    public static PIDCoefficients pid = new PIDCoefficients(0,0,0);

    BasicPID controller = new BasicPID(pid);  // create aPID controller

    @Override
    public void initAuto(HardwareMap hwMap) {
        arm = hwMap.get(DcMotor.class, "arm"); // hardware map arm
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // set modes for arm
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {
        arm.setPower(0);  // shutdown

    }
    public void resetArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // reset the arm's modes
    }
    public double getArmPos(){
        return arm.getCurrentPosition();  // return the arm's position
    }
    // Method to convert encoder ticks to degrees

//    TODO: Check to make sure this method works.
    public static double ticksToDegrees(int ticks) {
        // Calculate the degrees per tick
        double degreesPerTick = 360.0 / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
        // Convert ticks to degrees
        return ticks * degreesPerTick;
    }
    public void armPID(int ref){  // use PID to go to reference
        if (ref < maxarmconstraint && ref > minarmconstraint) {
            arm.setPower(controller.calculate(ticksToDegrees(ref), getArmPos()));
            // as long as the reference value is good, use PID to calculate the power for the arm to get to the reference value
        } else {
            shutdown();  // otherwise don't do anything
        }
    }

    public void armPIDTicks(int ref){  // has arm go to position but no constraints on if the position is good
        arm.setPower(controller.calculate(ref,getArmPos()));
    }

    public void setArmStates(ArmStates armStates){  // set all the arm states for the arm
        switch (armStates){
            case ScoreHighBasket:  // score in the high basket
                armPID(highbasket);
                break;
            case ScoreLowBasket:  // score in the low basket
                armPID(lowbasket);
                break;
            case Down:  // stay down
                armPID(0);
                break;
        }
    }
    public double getArmError(){  // find the difference between the arm's current position and 0.  Used to determine if the arm is at a resting position
        return ref - getArmPos();
    }

    // name for the arm's states
    public enum ArmStates{
        ScoreHighBasket,
        ScoreLowBasket,
        Down
    }
}
