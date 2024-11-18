package org.firstinspires.ftc.teamcode.Opmode.Teleop;


import org.firstinspires.ftc.teamcode.CommandFrameWork.BaseTele;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.RobotRelative;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "\uD83D\uDE08")
public class TeleOp extends BaseTele {
    @Override
    public Command setUpTele(CommandScheduler commandScheduler) {// this is where the meat of the code is


        return new MultipleCommand(new RobotRelative(robot.driveTrain,robot.gamepad1)); // drivetrain
    }
}
