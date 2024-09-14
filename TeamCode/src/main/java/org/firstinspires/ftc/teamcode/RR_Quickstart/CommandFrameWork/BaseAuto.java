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
//    protected PathInit pathInit;
    protected Trajectory purplepixel,turn,stack,prepreyellowpixel,preyellowpixel,yellowpixel,postyellowpixel,preprestack2,predeposit,prestack2,stack2,deposit,park;

    protected Trajectory bpurplepixel,bturn,bstack,bprepreyellowpixel,bpreyellowpixel,byellowpixel,bpostyellowpixel,bpreprestack2,bpredeposit,bprestack2,bstack2,bdeposit,bpark;

    @Override
    public void runOpMode() throws InterruptedException {
        groups = new ScoringCommandGroups(robot.slides, robot.intake);


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
    public abstract Command runAutoMid(CommandScheduler scheduler);
    public abstract Command runAutoRight(CommandScheduler scheduler);
    public abstract Command runAutoLeft(CommandScheduler scheduler);
    public FollowPath RoadRunnerPath(Trajectory traj){
        return new FollowPath(traj,robot,robot.dashboard);
    }
    public void setRobot(){
        robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
    }
}