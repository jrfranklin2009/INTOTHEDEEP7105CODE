package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

public class Arm extends Subsystem {

    public static int ref = 0;
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

    public void armPID(int ref){
        arm.setPower(controller.calculate(ref,getArmPos()));
    }

    public void setArmStates(ArmStates armStates){
        switch (armStates){
            case Up:
                armPID(ref);
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
        Up,
        Down
    }
}
