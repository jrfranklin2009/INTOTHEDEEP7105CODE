package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;

import org.firstinspires.ftc.teamcode.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmExtension;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmRotation;


public abstract class BaseTele extends LinearOpMode {
    protected Robot robot;
    protected ScoringCommandGroups groups;

    @Override
    public void runOpMode() throws InterruptedException {

        robot =new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2);
        groups =new ScoringCommandGroups( robot.intake, robot.armExtension, robot.armRotation);
//        ScoringCommandGroups groups = new ScoringCommandGroups(robot.slides, robot.intake, robot.arm);
//        RobotRelative robotRelative = new RobotRelative(robot.driveTrain,robot.gamepad1);


        waitForStart();
        while (opModeIsActive()){;
         //   telemetry.addData("hang position", HangingMechanism.getLeadScrewOnePos());
//            robot.driveTrain.mecanumDrive.setWeightedDrivePower(new Pose2d(
//                    -robot.gamepad1.getLeft_stick_y(),
//                    -robot.gamepad1.getLeft_stick_x(),
//                    -robot.gamepad1.getRight_stick_x()));

            robot.gamepad1.whenRightBumperPressed(groups.moveArmExtensionPID(ArmExtension.ArmExtensionStates.MaxExtension));
            robot.gamepad1.whenLeftBumperPressed(groups.moveArmExtensionPID(ArmExtension.ArmExtensionStates.FullyContracted));
            robot.gamepad1.whenTrianglePressed(groups.moveArmRotationPID(ArmRotation.ArmRotationStates.HighBucket_Back));
            robot.gamepad1.whenSquarePressed(groups.moveArmRotationPID(ArmRotation.ArmRotationStates.PickUp));
            robot.updateTele();


        }

    }

    public abstract Command setUpTele(CommandScheduler commandScheduler);
}
