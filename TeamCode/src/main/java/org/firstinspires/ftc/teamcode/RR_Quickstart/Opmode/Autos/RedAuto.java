package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode.Autos;

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
                .build();

                // clip specimen

        Trajectory trajectory1 = robot.driveTrain.mecanumDrive.trajectoryBuilder(new Pose2d(32, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50,0, Math.toRadians(0)))
                .build();

        Trajectory trajectory2 = robot.driveTrain.mecanumDrive.trajectoryBuilder(new Pose2d(50,0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(23, -40, Math.toRadians(90)))
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

        runpath = new MultipleCommand(RoadRunnerPath(trajectory).addNext(RoadRunnerPath(trajectory1))
                .addNext(RoadRunnerPath(trajectory2))
                .addNext(RoadRunnerPath(trajectory3))
                .addNext(RoadRunnerPath(trajectory4))
                .addNext(RoadRunnerPath(trajectory5))
                .addNext(RoadRunnerPath(trajectory6))
                .addNext(RoadRunnerPath(trajectory7))
                .addNext(RoadRunnerPath(trajectory8))
                .addNext(RoadRunnerPath(trajectory9)));

        // run the trajectories
        return runpath;

    }
}
