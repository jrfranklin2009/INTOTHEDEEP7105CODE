package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.BaseAuto;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.Delay;
import org.firstinspires.ftc.teamcode.RR_Quickstart.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedAuto extends BaseAuto {
    @Override
    public Command runAuto(CommandScheduler scheduler) {
        Command runpath;

        robot.driveTrain.mecanumDrive.setPoseEstimate(new Pose2d(0,-63,Math.toRadians(270)));

//        Trajectory trajectory = robot.driveTrain.mecanumDrive.trajectoryBuilder(new Pose2d(0, -63, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(0, -32, Math.toRadians(270)))
//                .build();

        TrajectorySequence trajectory = robot.driveTrain.mecanumDrive.trajectorySequenceBuilder(new Pose2d(0, -63, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(0, -32, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(155)))
                .lineToLinearHeading(new Pose2d(-60,-63, Math.toRadians(270)))
                .build();

                // clip specimen

        Trajectory trajectory1 = robot.driveTrain.mecanumDrive.trajectoryBuilder(new Pose2d(0, -32, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(270)))
                .build();

        Trajectory trajectory2 = robot.driveTrain.mecanumDrive.trajectoryBuilder(new Pose2d(0,-50, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(270)))
                .build();

                // pick up first yellow sample
        Trajectory trajectory3 = robot.driveTrain.mecanumDrive.trajectoryBuilder(new Pose2d(23, -40, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(53, -53, Math.toRadians(315)))
                .build();

                // depositing sample
        Trajectory trajectory4 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory3.end())
                .lineToLinearHeading(new Pose2d(23 -49, Math.toRadians(90)))
                .build();

                // pick up second yellow sample

        Trajectory trajectory5 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading(new Pose2d(53, -53, Math.toRadians(315)))
                .build();

                // depositing sample

        Trajectory trajectory6 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory5.end())
                .lineToLinearHeading(new Pose2d(23, -58, Math.toRadians(90)))
                .build();

                // pick up third yellow sample

        Trajectory trajectory7 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory6.end())
                .lineToLinearHeading(new Pose2d(53, -53, Math.toRadians(315)))
                .build();

                // depositing sample

        Trajectory trajectory8 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory7.end())
                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(0)))
                .build();

        Trajectory trajectory9 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory8.end())
                .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(0)))
                .build();

                // park

        runpath = new MultipleCommand(RoadRunnerPathSequence(trajectory))
                .addNext(new Delay(1));
//                .addNext(RoadRunnerPath(trajectory3))
//                .addNext(RoadRunnerPath(trajectory4))
//                .addNext(RoadRunnerPath(trajectory5))
//                .addNext(RoadRunnerPath(trajectory6))
//                .addNext(RoadRunnerPath(trajectory7))
//                .addNext(RoadRunnerPath(trajectory8))
//                .addNext(RoadRunnerPath(trajectory9)));

        // run the trajectories
        return runpath;

    }
}
