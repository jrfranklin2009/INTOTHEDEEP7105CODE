
package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestTridentArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor TurnTridentArm;
        DcMotor ExtendTridentArm;

        Boolean isReverse = false;

        init();

        TurnTridentArm =hardwareMap.get(DcMotor.class, "TurnTridentArm");
        ExtendTridentArm = hardwareMap.get(DcMotor.class, "ExendTridentArm");

        waitForStart();
        while(opModeIsActive()){
//            ExtendTridentArm.setPower(0.3*gamepad1.left_trigger);
//            TurnTridentArm.setPower(0.3*gamepad1.right_trigger);
//            ExtendTridentArm.setPower(-0.3*gamepad1.left_bumper);
//            TurnTridentArm.setPower(-0.3*gamepad1.right_bumper);

            if (gamepad1.a){
                isReverse = true;
            }
            if (gamepad1.b){
                isReverse = false;
            }

            if (!isReverse){
                ExtendTridentArm.setPower(0.3*gamepad1.left_trigger);
                TurnTridentArm.setPower(0.3*gamepad1.right_trigger);
            }
            if (isReverse){
                ExtendTridentArm.setPower(-0.3* gamepad1.left_trigger);
                TurnTridentArm.setPower(-0.3* gamepad1.right_trigger);
            }
        }
    }
}
