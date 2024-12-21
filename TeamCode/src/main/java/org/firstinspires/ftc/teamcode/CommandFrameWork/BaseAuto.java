package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.FollowPathSequence;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlidesMultiThread;
import org.firstinspires.ftc.teamcode.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.ClipMech.ClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class BaseAuto extends LinearOpMode {
    protected Robot robot;
    public ScoringCommandGroups groups;
//    MoveVerticalSlidesMultiThread moveSlides;
Command runpath;
    TrajectorySequence trajectory;
    @Override
    public void runOpMode() throws InterruptedException {

        setRobot();

        groups = new ScoringCommandGroups(robot.intake, robot.verticalslides, robot.horizontalslides,robot.clipmech, robot.hang, this);
//        moveSlides = new MoveVerticalSlidesMultiThread()
        robot.getScheduler().forceCommand(groups.initRobot());
        while (opModeInInit()){
            robot.driveTrain.mecanumDrive.setPoseEstimate(new Pose2d(-15.8, 59.7, Math.toRadians(90)));
             trajectory= robot.driveTrain.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-15.8, 59.7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-15.8, 46, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-48,46, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-52.7, 52.7, Math.toRadians(45) ))
                .lineToLinearHeading(new Pose2d(-59.9, 46, Math.toRadians(90))  )
                .lineToLinearHeading(new Pose2d(-52.7, 52.7, Math.toRadians(45) ))
                .lineToLinearHeading(new Pose2d(-48, 46, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(-52.7, 52.7, Math.toRadians(45) ))
                .build();
            runpath = new MultipleCommand(RoadRunnerPathSequence(trajectory)).addNext(groups.moveClipMechanismsOut(0, ClipMech.ArmStates.Out_The_Way,
                    HorizontalSlides.HorizontalSlideStates.Half_Out,190, JohnsIntake.ArmStates.preauto_clip));
        }

        waitForStart();
//        robot.getScheduler().forceCommand(runAuto(robot.getScheduler()));
        robot.getScheduler().forceCommand(runpath);
        while (opModeIsActive() && !isStopRequested()){
            robot.update();
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
        robot.shutdown();
    }
//    public abstract Command runAuto(CommandScheduler scheduler);
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