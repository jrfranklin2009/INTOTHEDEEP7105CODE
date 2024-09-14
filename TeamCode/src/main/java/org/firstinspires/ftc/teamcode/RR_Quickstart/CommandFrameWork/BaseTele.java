package org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Robot;


public class BaseTele extends LinearOpMode {
    protected Robot robot;
    protected ScoringCommandGroups groups;

    @Override
    public void runOpMode() throws InterruptedException {

        robot =new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2);
        groups =new ScoringCommandGroups(robot.slides, robot.intake, robot.arm);

        waitForStart();
        while (opModeIsActive()){
            robot.updateTele();
        }

    }
}
