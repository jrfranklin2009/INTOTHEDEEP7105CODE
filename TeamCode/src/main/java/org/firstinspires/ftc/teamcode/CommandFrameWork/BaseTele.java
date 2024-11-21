package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntakeJohn;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;
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

//        while (opModeInInit()){
//            setStartYOffSet();
//            setStartXOffSet();
//        }
//        ScoringCommandGroups groups = new ScoringCommandGroups(robot.slides, robot.intake, robot.arm);

        waitForStart();
        robot.driveTrain.setRR_PinPoint(setXPos(),setYPos(),setHeading());
        while (opModeIsActive()){
            robot.getScheduler().forceCommand(setUpTele(robot.getScheduler()));
            moveIntakeJohn.periodic();
            robot.verticalslides.updatePos(robot.gamepad2);

            robot.gamepad2.whenRightBumperPressed(moveVerticalSlides);
            robot.gamepad2.whenLeftBumperPressed(moveVerticalSlides);

            robot.driveTrain.RobotRelative(robot.gamepad1);
            robot.updateTele();


        }

    }

    public abstract Command setUpTele(CommandScheduler commandScheduler);
    public abstract double setYPos();
    public abstract double setXPos();
    public abstract double setHeading();
}
