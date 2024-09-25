package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism;

import static org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm.COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm.maxarmconstraint;
import static org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm.minarmconstraint;
import static org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm.pid;
import static org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm.ref;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

public class HangingMechanism extends Subsystem {

    public static int hangFirstLevel = 500;

    BasicPID controller = new BasicPID(pid);

    DcMotor LeadScrewOne;
    DcMotor LeadScrewTwo;
    ServoImplEx TurnScrewOne;
    ServoImplEx TurnScrewTwo;

    @Override
    public void initAuto(HardwareMap hwMap) {

        LeadScrewOne = hwMap.get(DcMotor.class, "LeadScrewOne");
        LeadScrewTwo = hwMap.get(DcMotor.class, "LeadScrewTwo");

        LeadScrewTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeadScrewTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeadScrewOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeadScrewOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TurnScrewOne = hwMap.get(ServoImplEx.class, "TurnScrewOne");
        TurnScrewTwo = hwMap.get(ServoImplEx.class, "TurnScrewTwo");

    }

    @Override
    public void initTeleop(HardwareMap hwMap){

        LeadScrewOne = hwMap.get(DcMotor.class, "LeadScrewOne");
        LeadScrewTwo = hwMap.get(DcMotor.class, "LeadScrewTwo");

        LeadScrewTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeadScrewTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeadScrewOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeadScrewOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TurnScrewOne = hwMap.get(ServoImplEx.class, "TurnScrewOne");
        TurnScrewTwo = hwMap.get(ServoImplEx.class, "TurnScrewTwo");

    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {
        LeadScrewOne.setPower(0);
        LeadScrewTwo.setPower(0);

        TurnScrewTwo.setPwmDisable();
        TurnScrewOne.setPwmDisable();

    }

    public double getLeadScrewOnePos() {return LeadScrewOne.getCurrentPosition();}

    public double getLeadScrewTwoPos() {return LeadScrewTwo.getCurrentPosition();}

    public static double ticksToDegrees(int ticks) {
        // Calculate the degrees per tick
        double degreesPerTick = 360.0 / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
        // Convert ticks to degrees
        return ticks * degreesPerTick;
    }

    public void LeadScrewTwoPID(int ref) {
        if (ref < maxarmconstraint && ref > minarmconstraint) {
            LeadScrewTwo.setPower(controller.calculate(ticksToDegrees(ref), getLeadScrewTwoPos()));
        } else {
            LeadScrewTwo.setPower(0);
        }
    }

    public void LeadScrewOnePID(int ref){
        if (ref < maxarmconstraint && ref > minarmconstraint){
            LeadScrewOne.setPower(controller.calculate(ticksToDegrees(ref), getLeadScrewOnePos()));
        }
        else {
            LeadScrewOne.setPower(0);
        }

    }

    public void LeadSrewOnePIDTicks(int ref) {
        LeadScrewOne.setPower(controller.calculate(ref, getLeadScrewOnePos()));
    }

    public void LeadScrewTwoPIDTicks(int ref) {
        LeadScrewTwo.setPower(controller.calculate(ref,getLeadScrewTwoPos()));
    }

    public void setLeadScrewStates(LeadScrewStates leadscrewstates){
        switch(leadscrewstates){
            case Down:
                LeadScrewOnePID(0);
                LeadScrewTwoPID(0);
                break;
            case HangFirstLevel:
                LeadScrewTwoPID(hangFirstLevel);
                LeadScrewOnePID(hangFirstLevel);
                break;
        }
    }

    public void setLeadScrewTurnStates(LeadScrewTurnStates leadscrewturnstates){
        switch(leadscrewturnstates){
            case Normal:
                TurnScrewOne.setPosition(0);
                TurnScrewTwo.setPosition(0);
                break;
            case Hang:
                TurnScrewOne.setPosition(0.2);
                TurnScrewTwo.setPosition(0.2);
            case Coast:
                TurnScrewTwo.setPwmDisable();
                TurnScrewOne.setPwmDisable();
        }
    }

    public double getLeadScrewOneError() {return ref-getLeadScrewOnePos();}

    public double getLeadScrewTwoError() {return ref - getLeadScrewTwoPos();}

    public enum LeadScrewStates{
        Down,
        HangFirstLevel
    }

    public enum LeadScrewTurnStates {
        Normal,
        Hang,
        Coast
    }
}
