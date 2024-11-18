package org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class FollowPathSequence extends Command {
    Robot robot;

    Dashboard dashboard;

    TrajectorySequence traj;

    public FollowPathSequence(TrajectorySequence traj, Robot robot, Dashboard dashboard){
        this.robot = robot;
        this.dashboard = dashboard;
        this.traj = traj;
    }


    @Override
    public void init() {
        robot.driveTrain.followTrajectorySequenceAsync(traj);
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean completed() {
        return !robot.driveTrain.mecanumDrive.isBusy();
    }

    @Override
    public void shutdown() {
        robot.driveTrain.shutdown();
    }
}
