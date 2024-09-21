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

        TrajectorySequence trajectory = robot.driveTrain.mecanumDrive.trajectorySequenceBuilder(new Pose2d(0, -63, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(0, -32, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(155)))
                .lineToLinearHeading(new Pose2d(-58,-63.5, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-55,-50, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-56.5,-49, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-56.5,-30, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-56.5,-31, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-58,-63.5, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-59,-61, Math.toRadians(270)))
                .build();


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
