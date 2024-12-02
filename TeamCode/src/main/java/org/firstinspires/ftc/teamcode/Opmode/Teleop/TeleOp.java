package org.firstinspires.ftc.teamcode.Opmode.Teleop;


import org.firstinspires.ftc.teamcode.CommandFrameWork.BaseTele;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.ClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "\uD83D\uDE08")
public class TeleOp extends BaseTele {
    @Override
    public Command setUpTele(CommandScheduler commandScheduler) {// this is where the meat of the code is
        robot.gamepad1.whenCrossPressed(groups.fullExtendHorizontalSLides());
        robot.gamepad1.whenSquarePressed(groups.bringInHorizontalSLidesBetter());
        robot.gamepad1.whenTrianglePressed(groups.extendHorizontalSLides());

        robot.gamepad2.whenDPadUpPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Clippity_Clappity_Clickity_Click));
        robot.gamepad2.whenDPadLeftPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Almost_Down));
        robot.gamepad2.whenDPadRightPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.READY));
        robot.gamepad2.whenDPadDownPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Down));
        robot.gamepad2.whenCrossPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Out_The_Way));

        robot.gamepad1.whenDPadUpPressed(groups.armOutBack());
        robot.gamepad1.whenDPadRightPressed(groups.moveArmJohn(JohnsIntake.ArmStates.parallel));
        robot.gamepad1.whenDPadDownPressed(groups.moveArmJohn(JohnsIntake.ArmStates.forward));

        robot.gamepad1.whenLeftBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.unclamp));
        robot.gamepad1.whenRightBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.clamp));
        return new MultipleCommand(new RobotRelative(robot.driveTrain,robot.gamepad1)); // drivetrain
    }

    @Override
    public double setYPos() {
        return -46;
    }

    @Override
    public double setXPos() {
        return -23;
    }

    @Override
    public double setHeading() {
        return 270;
    }
}
