package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

/** This is code for the hanging mechanism.
 * Includes turning of the lead screws and the lead screws themselves
 */
@Config
public class HangingMechanism extends Subsystem {

    //TODO test this position for the first level hang
    public static int hangFirstLevel = 500;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;

    public static PIDCoefficients pid = new PIDCoefficients(Kp,Ki,Kd);

    BasicPID controller = new BasicPID(pid);  // PID controller

    static DcMotor LeadScrewOne;  // motors for lead screws
    static DcMotor LeadScrewTwo;
    public ServoImplEx TurnScrewOne;  // the servos for turning the lead screws
    public ServoImplEx TurnScrewTwo;

    Telemetry telemetry;

    public HangingMechanism(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {

        LeadScrewOne = hwMap.get(DcMotor.class, "LeadScrewOne");  // hardware mapping lead screw motors
        LeadScrewTwo = hwMap.get(DcMotor.class, "LeadScrewTwo");
        TurnScrewOne = hwMap.get(ServoImplEx.class, "TurnScrewOne");  // hardware map lead screw servos
        TurnScrewTwo = hwMap.get(ServoImplEx.class, "TurnScrewTwo");
        TurnScrewTwo.setDirection(Servo.Direction.REVERSE);


        LeadScrewTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeadScrewTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // make it so they can use encoders
        LeadScrewOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeadScrewOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurnScrewTwo.setDirection(Servo.Direction.REVERSE);
        TurnScrewOne.setDirection(Servo.Direction.REVERSE);
//        setLeadScrewStates(LeadScrewStates.Down);
        setLeadScrewTurnStates(LeadScrewTurnStates.Normal);

        TurnScrewOne.setPwmEnable();
        TurnScrewTwo.setPwmEnable();



    }

//    @Override
//    public void initTeleop(HardwareMap hwMap){
//        LeadScrewOne = hwMap.get(DcMotor.class, "LeadScrewOne");
//        LeadScrewTwo = hwMap.get(DcMotor.class, "LeadScrewTwo");  // hardware mapping lead screw motors
//        TurnScrewOne = hwMap.get(ServoImplEx.class, "TurnScrewOne");
//        TurnScrewTwo = hwMap.get(ServoImplEx.class, "TurnScrewTwo");  // hardware mapping lead screw servos
//
//        LeadScrewTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LeadScrewTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // make it so they can use encoders
//        LeadScrewOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LeadScrewOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//
//
//    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {
        LeadScrewOne.setPower(0);
        LeadScrewTwo.setPower(0);
        // shut everything down
        TurnScrewTwo.setPwmDisable();
        TurnScrewOne.setPwmDisable();

    }

    public static double getLeadScrewOnePos() {return LeadScrewOne.getCurrentPosition();}  // get the position of the first lead screw motor

    public static double getLeadScrewTwoPos() {return LeadScrewTwo.getCurrentPosition();}  // get the position fo the second lead screw motor

//    public static double ticksToDegrees(int ticks) {
//        // Calculate the degrees per tick
////        double degreesPerTick = 360.0 / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
//        // Convert ticks to degrees
//        return ticks * degreesPerTick;
//    }

//    public void LeadScrewTwoPID(int ref) {
//        // use PID for the second lead screw
//        if (ref < maxarmconstraint && ref > minarmconstraint) {
//            LeadScrewTwo.setPower(controller.calculate(ticksToDegrees(ref), getLeadScrewTwoPos()));
//            // go to the reference position if it is within acceptable boundaries
//        } else {
//            LeadScrewTwo.setPower(0);  // otherwise don't do anything
//        }
//    }

//    public void LeadScrewOnePID(int ref){
//        // use PID for the first lead screw
//        if (ref < maxarmconstraint && ref > minarmconstraint){
//            LeadScrewOne.setPower(controller.calculate(ticksToDegrees(ref), getLeadScrewOnePos()));
//            // go to the reference position if it is within acceptable boundaries
//        }
//        else {
//            LeadScrewOne.setPower(0);  // otherwise don't do anything
//        }
//
//    }

    public void LeadScrewOnePIDTicks(int ref) {
        LeadScrewOne.setPower(controller.calculate(ref, getLeadScrewOnePos()));
        // use PID to set first lead screw to a position.  No constraints on what the position will be
    }

    public void LeadScrewTwoPIDTicks(int ref) {
        LeadScrewTwo.setPower(controller.calculate(ref,getLeadScrewTwoPos()));
        // use PID to set second lead screw to a position.  No constraints on what the position will be
    }

//    public void setLeadScrewStates(LeadScrewStates leadscrewstates){
//        // lead screw states
//        switch(leadscrewstates){
//            case Down:
//                // when not hanging
//                LeadScrewOnePID(0);
//                LeadScrewTwoPID(0);
//                break;
//            case HangFirstLevel:
//                // position for hanging
//                LeadScrewTwoPID(hangFirstLevel);
//                LeadScrewOnePID(hangFirstLevel);
//                break;
//        }
//    }

    public void setLeadScrewTurnStates(LeadScrewTurnStates leadscrewturnstates){
        // states for the rotating of the lead screw
        switch(leadscrewturnstates){
            case Normal:  // this is the default
                TurnScrewOne.setPosition(0);
                TurnScrewTwo.setPosition(0);
                break;
            case Hang:
                TurnScrewOne.setPosition(0.2);
                TurnScrewTwo.setPosition(0.2);  // hanging
                break;
            case Coast:
                TurnScrewTwo.setPwmDisable();  // set it to coast so the robot can reposition itself
                TurnScrewOne.setPwmDisable();
                break;
        }
    }

//    public double getLeadScrewOneError() {return ref-getLeadScrewOnePos();}  // get the difference between the first lead screw's position and zero.  Used to check completion of movement

//    public double getLeadScrewTwoError() {return ref - getLeadScrewTwoPos();}  // get the difference between the second lead screw's position and zero.  Used to check completion of movement

    // names for lead screw states
    public enum LeadScrewStates{
        Down,
        HangFirstLevel
    }

    // names for the turning of the lead screws
    public enum LeadScrewTurnStates {
        Normal,
        Hang,
        Coast
    }
}
