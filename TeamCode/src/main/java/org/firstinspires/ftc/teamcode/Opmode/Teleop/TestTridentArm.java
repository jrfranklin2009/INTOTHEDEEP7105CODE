
package org.firstinspires.ftc.teamcode.Opmode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class TestTridentArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor TurnTridentArm;
        DcMotor ExtendTridentArm;
        CRServoImplEx Intake;

        DcMotor LeftHang;
        DcMotor RightHang;

        Boolean isReverse = false;

        init();

        TurnTridentArm =hardwareMap.get(DcMotor.class, "TurnTridentArm");
        ExtendTridentArm = hardwareMap.get(DcMotor.class, "ExendTridentArm");
        Intake = hardwareMap.get(CRServoImplEx.class, "Intake");

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
                ExtendTridentArm.setPower(0.3*gamepad1.left_trigger+0.05);
                TurnTridentArm.setPower(0.3*gamepad1.right_trigger+0.05);
            }
            if (isReverse){
                ExtendTridentArm.setDirection(DcMotorSimple.Direction.REVERSE);
                TurnTridentArm.setDirection(DcMotorSimple.Direction.REVERSE);
                ExtendTridentArm.setPower(0.3*gamepad1.left_trigger+0.05);
                TurnTridentArm.setPower(0.3*gamepad1.right_trigger+0.05);;
            }

            if (gamepad1.right_bumper){
                Intake.setPower(0.75);
            }
        }
    }
}
