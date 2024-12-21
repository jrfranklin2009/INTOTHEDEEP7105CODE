package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.PinPoint_MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinPoint_MecanumDrive drive = new PinPoint_MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        drive.startIMUThread(this);

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.a){
                drive.leftRear.setPower(1); //leftfront
            } else if (gamepad1.b) {
                drive.leftFront.setPower(1);  //leftback
            } else if (gamepad1.y) {
                drive.rightRear.setPower(1);  //rightfront
            } else if (gamepad1.x){
                drive.rightFront.setPower(1);
            }

            drive.update();
            telemetry.addData("leftfront",drive.leftFront.getCurrentPosition());
            telemetry.addData("leftrear",drive.leftRear.getCurrentPosition());
            telemetry.addData("rightfront",drive.rightFront.getCurrentPosition());
            telemetry.addData("rightrear",drive.rightRear.getCurrentPosition());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
