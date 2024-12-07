package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.FollowPathSequence;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class BaseAuto extends LinearOpMode {
    protected Robot robot;
    public ScoringCommandGroups groups;
    @Override
    public void runOpMode() throws InterruptedException {

        setRobot();

        groups = new ScoringCommandGroups(robot.intake, robot.verticalslides, robot.horizontalslides,robot.clipmech,this);

        while (opModeInInit()){
        }

        waitForStart();
        robot.getScheduler().forceCommand(runAuto(robot.getScheduler()));

        while (opModeIsActive() && !isStopRequested()){
            robot.update();
//            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//            }
        }
        robot.shutdown();
    }
    public abstract Command runAuto(CommandScheduler scheduler);
    public FollowPath RoadRunnerPath(Trajectory traj){
        return new FollowPath(traj,robot,robot.dashboard);
    }

    public FollowPathSequence RoadRunnerPathSequence(TrajectorySequence traj){
        return new FollowPathSequence(traj,robot,robot.dashboard);
    }

//    public Command RoadRunnerPathSequenceBet(TrajectorySequence traj){
//        return robot.driveTrain.mecanumDrive.followTrajectorySequence(traj);
//    }

    public void setRobot(){
        robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
    }

}