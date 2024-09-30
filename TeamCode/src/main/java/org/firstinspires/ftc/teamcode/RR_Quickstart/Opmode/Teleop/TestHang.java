package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestHang extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor hang;
        DcMotor hang2;

        init();


        hang = hardwareMap.get(DcMotor.class,"LeadScrewOne");
        hang2 = hardwareMap.get(DcMotor.class,"LeadScrewTwo");

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                hang.setPower(1);
                hang2.setPower(1);
            } else if (gamepad1.b) {
                hang.setPower(-1);
                hang2.setPower(-1);
            } else {
                hang.setPower(0);
                hang2.setPower(0);
            }
        }

    }
}
