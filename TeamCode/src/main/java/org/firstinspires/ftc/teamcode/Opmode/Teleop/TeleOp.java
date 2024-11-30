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
//        robot.gamepad1.whenCrossPressed(groups.moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Fully_Out));
//        robot.gamepad1.whenSquarePressed(groups.moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Fully_In));

//        robot.gamepad1.whenLeftBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.unclamp));
//        robot.gamepad1.whenRightBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.clamp));

//        robot.gamepad1.whenDPadUpPressed(groups.moveArmJohn(JohnsIntake.ArmStates.outback));
//        robot.gamepad1.whenDPadDownPressed(groups.moveArmJohn(JohnsIntake.ArmStates.forward));

        return new MultipleCommand(new RobotRelative(robot.driveTrain,robot.gamepad1)); // drivetrain
    }
}
// DONT USE !!!!!!!!! wait for philip