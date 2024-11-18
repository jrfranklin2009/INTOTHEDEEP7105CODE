package org.firstinspires.ftc.teamcode.Opmode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class AutoNoScheduler extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

        init();

        mecanumDrive.setPoseEstimate(new Pose2d(63, 0, Math.toRadians(0)));

        TrajectorySequence trajectory = mecanumDrive.trajectorySequenceBuilder(new Pose2d(63, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50,0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(23, -40, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(53, -53, Math.toRadians(315)))
                .build();


        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive.followTrajectorySequence(trajectory);


        }
    }
}
