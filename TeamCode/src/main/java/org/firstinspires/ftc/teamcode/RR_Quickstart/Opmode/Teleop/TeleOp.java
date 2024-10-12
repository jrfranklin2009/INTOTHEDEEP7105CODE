package org.firstinspires.ftc.teamcode.RR_Quickstart.Opmode.Teleop;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.BaseTele;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "\uD83D\uDE08")
public class TeleOp extends BaseTele {
    @Override
    public Command setUpTele(CommandScheduler commandScheduler) {// this is where the meat of the code is

        robot.gamepad1.whenCrossPressed(groups.moveArmPID(Arm.ArmStates.ScoreLowBasket));
        robot.gamepad1.whenTrianglePressed(groups.turnArmPID(Arm.TurnStates.Vertical));

        return new MultipleCommand(new RobotRelative(robot.driveTrain,robot.gamepad1)); // drivetrain
    }
}
