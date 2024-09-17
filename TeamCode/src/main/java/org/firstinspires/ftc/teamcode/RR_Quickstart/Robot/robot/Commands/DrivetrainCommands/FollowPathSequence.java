package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.RR_Quickstart.trajectorysequence.TrajectorySequence;


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
    }

    @Override
    public void periodic() {
        robot.driveTrain.followTrajectorySequenceAsync(traj);
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
