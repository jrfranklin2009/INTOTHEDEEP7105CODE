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
        robot.driveTrain.followTrajectoryAsync(traj);
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
