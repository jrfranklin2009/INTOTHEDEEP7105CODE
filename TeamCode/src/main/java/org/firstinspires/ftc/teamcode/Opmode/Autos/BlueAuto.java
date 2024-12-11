package org.firstinspires.ftc.teamcode.Opmode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFrameWork.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlidesMultiThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous
public class BlueAuto extends BaseAuto {
    @Override
    public Command runAuto(CommandScheduler scheduler) {
        Command runpath;

        robot.driveTrain.mecanumDrive.setPoseEstimate(new Pose2d(-25, 63, Math.toRadians(90)));

        TrajectorySequence trajectory = robot.driveTrain.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-25, 63, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(0, 32, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(0,50, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(45, 30, Math.toRadians(-25)))
                .lineToLinearHeading(new Pose2d(58,63.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(55,50, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(56.5,49, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(56.5,30, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(56.5,31, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58,63.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(59,61, Math.toRadians(90)))
                .build();

        runpath = new MultipleCommand(RoadRunnerPathSequence(trajectory), moveSlides,groups.slidesSetPos())
                .addNext(new Delay(1));




        return runpath;
    }}