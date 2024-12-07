package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.HangingMechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

public class JohnHanging extends Subsystem {

    DcMotorEx rightHang,leftHang;

    public static double unfoldpower = -1, foldpower = 1;

    @Override
    public void initAuto(HardwareMap hwMap) {
        rightHang = hwMap.get(DcMotorEx.class, "rightHang");
        leftHang = hwMap.get(DcMotorEx.class, "leftHang");
        rightHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {

    }

    public void setRightHang(HangStates hangStates){
        switch(hangStates){
            case FOLD_UP:
                rightHang.setPower(foldpower);
                leftHang.setPower(foldpower);
//                rightHang.setTargetPosition(200);
//                rightHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightHang.setPower(0.3);
            break;
            case UNFOLD:
                rightHang.setPower(unfoldpower);
                leftHang.setPower(unfoldpower);
                break;
            case ZERO_POWER:
                rightHang.setPower(0);
                leftHang.setPower(0);
//                rightHang.setTargetPosition(0);
//                rightHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightHang.setPower(-0.3);
                break;
        }
    }

    public double getPos(){
        return rightHang.getCurrentPosition();
    }

    public enum HangStates{
        UNFOLD,
        FOLD_UP,
        ZERO_POWER
    }
}
