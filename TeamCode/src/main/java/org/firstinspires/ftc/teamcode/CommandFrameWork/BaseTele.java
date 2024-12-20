package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.ScoringCommandGroups;

import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntakeJohn;
import org.firstinspires.ftc.teamcode.Robot.robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.ClipMech.ClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;


public abstract class BaseTele extends LinearOpMode {
    protected Robot robot;
    protected ScoringCommandGroups groups;

    @Override
    public void runOpMode() throws InterruptedException {

        robot =new Robot(hardwareMap, Robot.OpMode.Teleop, gamepad1, gamepad2,this);
        groups =new ScoringCommandGroups(robot.intake, robot.verticalslides,robot.horizontalslides, robot.clipmech,robot.hang,this);
        MoveIntakeJohn moveIntakeJohn = new MoveIntakeJohn(robot.gamepad1, robot.intake, robot.horizontalslides);
        waitForStart();
        robot.driveTrain.startIMUThread(this);
//        robot.driveTrain.setRR_PinPoint(5,5,0);
        robot.driveTrain.resetPosAndHeading();
        robot.getScheduler().forceCommand(setUpTele(robot.getScheduler()));
        while (opModeIsActive()){
            moveIntakeJohn.periodic();
//            verticalSlides.periodic();
            robot.verticalslides.updatePos(robot.gamepad2,robot,groups);

            robot.gamepad1.whenCrossPressed(groups.bringInHorizontalSLidesBetter());
            robot.gamepad1.whenSquarePressed(groups.extendHorizontalSLides());
            robot.gamepad1.whenTrianglePressed(groups.fullExtendHorizontalSLides());

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            robot.gamepad2.whenRightTriggerPressed(groups.pullUp());
            robot.gamepad2.whenLeftTriggerPressed(groups.pullDown());


            robot.gamepad2.whenDPadUpPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Clippity_Clappity_Clickity_Click));
            robot.gamepad2.whenDPadLeftPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Almost_Down));
            robot.gamepad2.whenDPadRightPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.READY));
            robot.gamepad2.whenDPadDownPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Down));
            robot.gamepad2.whenCrossPressed(new MoveClipMech(robot.clipmech, ClipMech.ArmStates.Out_The_Way));

            robot.gamepad1.whenDPadUpPressed(groups.armOutBack());
            robot.gamepad1.whenDPadRightPressed(groups.clipClip());
            robot.gamepad1.whenDPadDownPressed(groups.moveArmJohn(JohnsIntake.ArmStates.parallel));
//        new MoveIndex(robot.clipmech, ClipMech.IndexState.LeftSide));
//        robot.gamepad1.whenDPadRightPressed(new MoveIndex(robot.clipmech, ClipMech.IndexState.RightSide));

            robot.gamepad1.whenLeftBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.unclamp));
            robot.gamepad1.whenRightBumperPressed(groups.moveGripper(JohnsIntake.GripperStates.clamp));

            robot.driveTrain.RobotRelative(robot.gamepad1);

//        robot.gamepad2.whenCirclePressed(groups.hangJohn(JohnHanging.HangStates.HANG_FULLY));
//        robot.gamepad2.whenTrianglePressed(groups.hangJohn(JohnHanging.HangStates.ZERO_POWER));

//            return new MultipleCommand(new RobotRelative(robot.driveTrain,robot.gamepad1)); // drivetrain


//           groups.slidesTeleop();
//            robot.gamepad2.whenLeftBumperPressed(groups.slidesTeleop());
//            robot.gamepad2.whenRightBumperPressed(new MoveVerticalSlidesMultiThread(robot.verticalslides, this));
//            robot.gamepad2.whenLeftBumperPressed(new MoveVerticalSlidesMultiThread(robot.verticalslides, this));

//            robot.driveTrain.RobotRelative(robot.gamepad1);//2880 1530
            robot.updateTele();
        }
    }
    public abstract Command setUpTele(CommandScheduler commandScheduler);
}
