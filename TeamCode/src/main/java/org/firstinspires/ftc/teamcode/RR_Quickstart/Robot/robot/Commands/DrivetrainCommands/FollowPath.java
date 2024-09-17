package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Dashboard;


public class FollowPath extends Command {
    Robot robot;

    Dashboard dashboard;

    Trajectory traj;

    public FollowPath(Trajectory traj,Robot robot, Dashboard dashboard){
        this.robot = robot;
        this.dashboard = dashboard;
        this.traj = traj;
    }


    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        //TODO check to see if this was the issue.
        robot.driveTrain.followTrajectory(traj);
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
