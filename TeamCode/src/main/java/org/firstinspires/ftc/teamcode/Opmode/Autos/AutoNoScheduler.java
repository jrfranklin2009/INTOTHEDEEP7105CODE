package org.firstinspires.ftc.teamcode.Opmode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.PinPoint.PinPoint_Odo;
import org.firstinspires.ftc.teamcode.drive.PinPoint_MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class AutoNoScheduler extends LinearOpMode {

    DcMotor TurnTridentArm;
    DcMotor ExtendTridentArm;
    CRServoImplEx bothintake;
    GoBildaPinpointDriver odo;
    DcMotor righthang,lefthang;

    @Override
    public void runOpMode() throws InterruptedException {


        TurnTridentArm =hardwareMap.get(DcMotor.class, "TurnTridentArm");
        ExtendTridentArm = hardwareMap.get(DcMotor.class, "ExendTridentArm");
        bothintake = hardwareMap.get(CRServoImplEx.class, "bothintake");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpointodo");
        righthang = hardwareMap.get(DcMotor.class,"righthang");
        lefthang = hardwareMap.get(DcMotor.class,"lefthang");


        righthang.setDirection(DcMotorSimple.Direction.REVERSE);
        PinPoint_MecanumDrive mecanumDrive = new PinPoint_MecanumDrive(hardwareMap, odo);
        init_loop();
        Pose2d startPose = new Pose2d(-9, -61, Math.toRadians(270));
        mecanumDrive.setPoseEstimate(startPose);


        Trajectory trajectory = mecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-9, -34, Math.toRadians(270)))
                .build();

        Trajectory trajectory1 = mecanumDrive.trajectoryBuilder(trajectory.end())
            .lineToLinearHeading(new Pose2d(-9, -51, Math.toRadians(270)))
            .build();



        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive.followTrajectory(trajectory);
//            mecanumDrive.followTrajectory(trajectory1);



//            mecanumDrive.updatePoseEstimate();
//            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

//            telemetry.addData("pinx", odo.getPosX());
//            telemetry.addData("piny", odo.getPosY());
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("pinheading", odo.getHeading());
//            telemetry.update();
            mecanumDrive.update();
        }

    }
}
