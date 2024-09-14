package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

public class Arm extends Subsystem {

    DcMotor arm;

    public static PIDCoefficients pid = new PIDCoefficients();

    PIDFController controller = new PIDFController(pid);

    public Arm(DcMotor arm){
        this.arm = arm;
    }

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

    public void armPID(){
        arm.setPower(controller.update(getArmPos()));
    }

    public enum ArmStates{
        Up,
        Down
    }
}
