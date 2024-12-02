package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.HangingMechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

public class JohnHanging extends Subsystem {

    DcMotorEx rightHang;

    @Override
    public void initAuto(HardwareMap hwMap) {
        rightHang = hwMap.get(DcMotorEx.class, "rightHang");
        rightHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {

    }


    public void setRightHang(HangStates hangStates){
        switch(hangStates){
            case HANG_FULLY:
                rightHang.setTargetPosition(200);
                rightHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHang.setPower(0.3);
            case NORMAL:
                rightHang.setTargetPosition(0);
                rightHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHang.setPower(-0.3);
        }
    }

    public enum HangStates{
        HANG_FULLY,
        NORMAL
    }
}
