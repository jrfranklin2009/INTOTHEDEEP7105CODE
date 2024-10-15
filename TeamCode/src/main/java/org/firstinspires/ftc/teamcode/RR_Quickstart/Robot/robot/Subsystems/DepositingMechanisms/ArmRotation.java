package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

/** This is code for the arm
 * Currently is just kooky arm.
 */
@Config
public class ArmRotation extends Subsystem {

    // Define the counts per revolution for your encoder
    public static final double COUNTS_PER_REVOLUTION = 751.8; // Adjust as needed

    // Define the gear ratio (output gear teeth / input gear teeth)
    public static final int GEAR_RATIO = 3, turnminarmconstraint =0, turnmaxarmconstraint = 600; // Adjust as needed

    public static int ref = 0, ninetydegrees = 200;

    DcMotor armrotation;

    public static PIDCoefficients pidturn = new PIDCoefficients(0.2,0.2,0.2);

    BasicPID turncontroller = new BasicPID(pidturn);

    @Override
    public void initAuto(HardwareMap hwMap) {
        armrotation = hwMap.get(DcMotor.class, "TurnTridentArm");
        resetArmRotation();
    }
    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {
        // shutdown
        armrotation.setPower(0);

    }

    public void resetArmRotation() {
        armrotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armrotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Method to convert encoder ticks to degrees
    public double getArmRotationPos(){return armrotation.getCurrentPosition();}

    //    TODO: Check to make sure this method works.
    public double ticksToDegrees(double ticks) {
        // Calculate the degrees per tick
        double degreesPerTick = 360.0 / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
        // Convert ticks to degrees
        return ticks * degreesPerTick;
    }

    public void armRotationPID(double ref){
        if (ref < turnmaxarmconstraint && ref > turnminarmconstraint){
            armrotation.setPower(turncontroller.calculate(ticksToDegrees(ref), getArmRotationPos()));
        } else{
            shutdown();
        }
    }

    public void setArmRotateStates(ArmRotationStates armRotationStates){
        switch (armRotationStates){
            case HighBucket_Back:
                ref = ninetydegrees;
                armRotationPID(ref);
                break;
            case LowBucket_Back:
                ref = ninetydegrees;
                armRotationPID(ref);
                break;
            case HighBucket_Front:
                ref = ninetydegrees;
                armRotationPID(ref);
                break;
            case LowBucket_Front:
                ref = ninetydegrees;
                armRotationPID(ref);
                break;
            case HighChamber:
                ref = ninetydegrees;
                armRotationPID(ref);
                break;
            case Low_Chamber:
                ref = ninetydegrees;
                armRotationPID(ref);
                break;
            case PrePickUp:
                ref = ninetydegrees;
                armRotationPID(ref);
                break;
            case PickUp:
                ref = 0;
                armRotationPID(ref);
                break;
        }
    }

    public double getArmRotateError() {
        return ref - getArmRotationPos();
    }

    public enum ArmRotationStates {
        HighBucket_Back,
        LowBucket_Back,
        HighBucket_Front,
        LowBucket_Front,
        HighChamber,
        Low_Chamber,
        PrePickUp,
        PickUp
    }
}