package org.firstinspires.ftc.teamcode.Opmode.Teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.CommandFrameWork.BaseTele;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleOpHang extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

        DcMotor righthang,lefthang;

        righthang = hardwareMap.get(DcMotor.class,"righthang");
        lefthang = hardwareMap.get(DcMotor.class,"lefthang");

        righthang.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

            mecanumDrive.setWeightedDrivePower(new Pose2d(gamepad1.right_stick_y,gamepad1.left_stick_x,
                    gamepad1.right_stick_x));

            righthang.setPower(gamepad1.left_stick_y);
            lefthang.setPower(gamepad1.left_stick_y);

        }


    }
}
