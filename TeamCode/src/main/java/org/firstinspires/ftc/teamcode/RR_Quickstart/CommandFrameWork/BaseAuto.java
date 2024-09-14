package org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands.FollowPath;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Robot;

public abstract class BaseAuto extends LinearOpMode {
    protected Robot robot;
    public ScoringCommandGroups groups;
    @Override
    public void runOpMode() throws InterruptedException {
        groups = new ScoringCommandGroups(robot.slides, robot.intake, robot.arm);


        while (opModeInInit()) {

        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            robot.update();
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
        robot.shutdown();
    }
    public abstract Command runAuto(CommandScheduler scheduler);
    public FollowPath RoadRunnerPath(Trajectory traj){
        return new FollowPath(traj,robot,robot.dashboard);
    }
    public void setRobot(){
        robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
    }
}