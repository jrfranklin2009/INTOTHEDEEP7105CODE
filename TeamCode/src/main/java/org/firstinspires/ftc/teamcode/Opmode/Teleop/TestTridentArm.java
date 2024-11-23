
package org.firstinspires.ftc.teamcode.Opmode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestTridentArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor TurnTridentArm;
        DcMotor ExtendTridentArm;
        CRServoImplEx bothintake;

        DcMotor LeftHang;
        DcMotor RightHang;

        Boolean isReverse = false;

        init();

        TurnTridentArm =hardwareMap.get(DcMotor.class, "TurnTridentArm");
        ExtendTridentArm = hardwareMap.get(DcMotor.class, "ExendTridentArm");
        bothintake = hardwareMap.get(CRServoImplEx.class, "bothintake");
        LeftHang = hardwareMap.get(DcMotor.class, "LeftHang");
        RightHang=hardwareMap.get(DcMotor.class, "RightHang");

        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.a){
                isReverse = true;
            }
            if (gamepad1.b){
                isReverse = false;
            }

            if (!isReverse){
                ExtendTridentArm.setPower(0.3*gamepad1.left_trigger);
                TurnTridentArm.setPower(0.5*gamepad1.right_trigger);
            }
            if (isReverse){
                ExtendTridentArm.setDirection(DcMotorSimple.Direction.REVERSE);
                TurnTridentArm.setDirection(DcMotorSimple.Direction.REVERSE);
                ExtendTridentArm.setPower(0.3*gamepad1.left_trigger);
                TurnTridentArm.setPower(0.3*gamepad1.right_trigger);;
            }

            if (gamepad1.right_bumper){
                bothintake.setPower(0.75);
            }


            if (gamepad1.left_bumper){
                bothintake.setPower(-0.75);
            }
        }

    }
}
