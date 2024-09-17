package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.BaseAuto;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.MultipleCommand;

@Autonomous
public class RedAuto extends BaseAuto {
    @Override
    public Command runAuto(CommandScheduler scheduler) {
        Command runpath;

        Trajectory trajectory = robot.driveTrain.mecanumDrive.trajectoryBuilder(new Pose2d(63, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(0)))
                // clip specimen
//                .lineToLinearHeading(new Pose2d(50,0, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(23, -40, Math.toRadians(90)))
                // pick up first yellow sample
//                .lineToLinearHeading(new Pose2d(53, -53, Math.toRadians(315)))
                // deposit sample
//                .lineToLinearHeading(new Pose2d(23, -49, Math.toRadians(90)))
                // pick up second yellow sample
//                .lineToLinearHeading(new Pose2d(53, -53, Math.toRadians(315)))
                // deposit second yellow sample
//                .lineToLinearHeading(new Pose2d(23, -58, Math.toRadians(90)))
                // pick up third yellow sample
//                .lineToLinearHeading(new Pose2d(53, -53, Math.toRadians(315)))
                // deposit third sample
//                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(0)))
                .build();

        Trajectory trajectory1 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory.end())
                .lineToLinearHeading(new Pose2d(50,0, Math.toRadians(0)))
                .build();

        Trajectory trajectory2 = robot.driveTrain.mecanumDrive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(23, -40, Math.toRadians(90)))
                .build();


        runpath = new MultipleCommand(RoadRunnerPath(trajectory).addNext(RoadRunnerPath(trajectory1))
                .addNext(RoadRunnerPath(trajectory2)));
        return runpath;
    }
}
