package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR_Quickstart.drive.SampleMecanumDrive;

public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
//        Robot robot = new Robot();


        waitForStart();

        mecanumDrive.setWeightedDrivePower(new Pose2d( // drive code
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));







    }
}
