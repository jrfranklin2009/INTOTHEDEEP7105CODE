package org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;


public abstract class BaseTele extends LinearOpMode {
    protected Robot robot;
    protected ScoringCommandGroups groups;

    @Override
    public void runOpMode() throws InterruptedException {

        robot =new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2, telemetry);
        groups =new ScoringCommandGroups( robot.intake, robot.arm, robot.hanging);
//        ScoringCommandGroups groups = new ScoringCommandGroups(robot.slides, robot.intake, robot.arm);
//        RobotRelative robotRelative = new RobotRelative(robot.driveTrain,robot.gamepad1);


        waitForStart();
        while (opModeIsActive()){;
         //   telemetry.addData("hang position", HangingMechanism.getLeadScrewOnePos());
            robot.driveTrain.mecanumDrive.setWeightedDrivePower(new Pose2d(
                    -robot.gamepad1.getLeft_stick_y(),
                    -robot.gamepad1.getLeft_stick_x(),
                    -robot.gamepad1.getRight_stick_x()));
            robot.updateTele();
            robot.gamepad1.whenCrossPressed(groups.HookOnBar(robot.gamepad1));
            robot.gamepad1.whenTrianglePressed(groups.ResetHanging(robot.gamepad1));

        }

    }

    public abstract Command setUpTele(CommandScheduler commandScheduler);
}
