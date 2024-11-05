package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntakeJohn;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmExtension;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmRotation;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;


public abstract class BaseTele extends LinearOpMode {
    protected Robot robot;
    protected ScoringCommandGroups groups;

    @Override
    public void runOpMode() throws InterruptedException {

        robot =new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2);
        groups =new ScoringCommandGroups(robot.intake, robot.verticalslides,robot.horizontalslides);
        MoveIntakeJohn moveIntakeJohn = new MoveIntakeJohn(robot.gamepad1, robot.intake);
        MoveVerticalSlides moveVerticalSlides = new MoveVerticalSlides(robot.verticalslides);
//        ScoringCommandGroups groups = new ScoringCommandGroups(robot.slides, robot.intake, robot.arm);

        waitForStart();
        while (opModeIsActive()){
            moveIntakeJohn.periodic();

            robot.gamepad2.whenRightBumperPressed(moveVerticalSlides);
            robot.gamepad2.whenLeftBumperPressed(moveVerticalSlides);

            robot.driveTrain.RobotRelative(robot.gamepad1);

//            robot.gamepad1.whenDPadUpPressed(groups.moveArmJohn(JohnsIntake.ArmStates.outback));
//            robot.gamepad1.whenDPadDownPressed(groups.moveArmJohn(JohnsIntake.ArmStates.forward));

//            robot.gamepad1.whenLeftTriggerPressed(groups.moveIntakeJohn(robot.gamepad1, JohnsIntake.IntakeStates.outtake));
//            robot.gamepad1.whenRightTriggerPressed(groups.moveIntakeJohn(robot.gamepad1, JohnsIntake.IntakeStates.intake));

//            robot.gamepad1.whenLeftBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.unclamp));
//            robot.gamepad1.whenRightBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.clamp));
            robot.updateTele();


        }

    }

    public abstract Command setUpTele(CommandScheduler commandScheduler);
}
