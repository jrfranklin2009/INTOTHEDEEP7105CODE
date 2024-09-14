package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.BaseTele;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends BaseTele {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2);
        ScoringCommandGroups groups = new ScoringCommandGroups(robot.slides, robot.intake, robot.arm);
        RobotRelative robotRelative = new RobotRelative(robot.driveTrain,robot.gamepad1);

        waitForStart();
        robot.gamepad1.whenRightBumperPressed(groups.intakeSample(robot.gamepad1));
        robot.gamepad1.whenLeftBumperPressed(groups.outtakeSample(robot.gamepad1));
        robot.gamepad1.whenSquarePressed(groups.outtakeSpecimen(robot.gamepad1));
        robotRelative.periodic();
    }
}
