package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.BaseTele;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends BaseTele {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2);
        Intake intake = new Intake();


        waitForStart();

        mecanumDrive.setWeightedDrivePower(new Pose2d( // drive code
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));


        robot.gamepad1.whenRightBumperPressed(groups.intakeSample(robot.gamepad1));
        robot.gamepad1.whenLeftBumperPressed(groups.outtakeSample(robot.gamepad1));
        robot.gamepad1.whenSquarePressed(groups.outtakeSpecimen(robot.gamepad1));







    }
}
