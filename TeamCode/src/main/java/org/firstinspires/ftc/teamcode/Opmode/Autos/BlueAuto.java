package org.firstinspires.ftc.teamcode.Opmode.Autos;

import static org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides.lowbasket;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFrameWork.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlidesMultiThread;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.ClipMech.ClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous
public class BlueAuto extends BaseAuto {
    @Override
    public Command runAuto(CommandScheduler scheduler) {
        Command runpath;

        robot.driveTrain.mecanumDrive.setPoseEstimate(new Pose2d(-15.8, -59.7, Math.toRadians(90)));

        TrajectorySequence trajectory = robot.driveTrain.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-15.8, -59.7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-15.8, -46, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-48,-46, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-52.7, -52.7, Math.toRadians(45) ))
                .lineToLinearHeading(new Pose2d(-59.9, -46, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-52.7, -52.7, Math.toRadians(45) ))
                .lineToLinearHeading(new Pose2d(-48, -46, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(-52.7, -52.7, Math.toRadians(45) ))
                .build();

        runpath = new MultipleCommand(RoadRunnerPathSequence(trajectory),
                groups.moveClipMechanismsOut(0, ClipMech.ArmStates.Out_The_Way,
                HorizontalSlides.HorizontalSlideStates.Half_Out,190, JohnsIntake.ArmStates.preauto_clip))
                .addNext(groups.moveArmJohn(JohnsIntake.ArmStates.forward));
        return runpath;
    }

}