package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;

import org.firstinspires.ftc.teamcode.Robot.robot.Robot;

public abstract class BaseTele extends LinearOpMode {
    protected Robot robot;
    protected ScoringCommandGroups groups;

    @Override
    public void runOpMode() throws InterruptedException {

        robot =new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2);
        groups =new ScoringCommandGroups(robot.intake, robot.extention, robot.rotation );
        ScoringCommandGroups groups = new ScoringCommandGroups(robot.intake, robot.extention, robot.rotation);

        waitForStart();
//        setUpTele()



        }

    public abstract Command setUpTele(CommandScheduler commandScheduler);
}


