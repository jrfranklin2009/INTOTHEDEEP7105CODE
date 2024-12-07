package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;

import org.firstinspires.ftc.teamcode.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.CloseThread;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveArmJohn;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveGripper;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHang;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHorizontalwithEncoder;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHorizontalwithEncoderOut;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlidesMultiThread;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.VerticalSlidesHoldPos;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.ClipMech.ClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.HangingMechanism.JohnHanging;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;

/** This contains all of the commands used in our robot.
 * Most commands are used in tele.
 */

public class ScoringCommandGroups {
    JohnsIntake intake;  // intake

//    ArmExtension armExtension;  // scoring mechanism

//    ArmRotation armRotation;

//    HangingMechanism hangingMechanism;  // hanging

    ClipMech clipmech;
//    JohnHanging hang;

    HorizontalSlides horizontalSlides;

    VerticalSlides verticalslides;

    LinearOpMode opMode;

    public ScoringCommandGroups(JohnsIntake intake, VerticalSlides verticalslides, HorizontalSlides horizontalslides, ClipMech clipmech, LinearOpMode opMode) {
        this.intake = intake;
        this.verticalslides = verticalslides;
        this.horizontalSlides = horizontalslides;
        this.clipmech = clipmech;
        this.opMode = opMode;
    }
    public Command slidesBetter(){
        return new MoveVerticalSlidesMultiThread(verticalslides,opMode).addNext(new CloseThread(verticalslides)).addNext(new VerticalSlidesHoldPos(verticalslides));
    }

    public Command armOutBack(){
        return new MultipleCommand(moveArmJohn(JohnsIntake.ArmStates.outback),moveGripper(JohnsIntake.GripperStates.unclamp));
    }

    public Command moveArmJohn(JohnsIntake.ArmStates armStates){
        return new MoveArmJohn(this.intake, armStates);
    }


    public Command moveGripper(JohnsIntake.GripperStates gripperStates){
        return new MoveGripper(this.intake, gripperStates);
    }

    public Command movevertSlides(){
        return new MoveVerticalSlides(this.verticalslides);
    }

    public Command moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates horizontalslidestates){
        return new MoveHorizontalSlides(this.horizontalSlides,horizontalslidestates);
    }

//    public Command extendHorizontalSLides(){
//        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.READY),new Delay(.1).addNext(moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Half_Out)),new Delay(.3).addNext(moveArmJohn(JohnsIntake.ArmStates.forward)));
//    }

    public Command fullExtendHorizontalSLides(){
        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.READY),new Delay(.1).addNext(new MoveHorizontalwithEncoderOut(horizontalSlides,HorizontalSlides.HorizontalSlideStates.Fully_Out,230)).addNext(moveArmJohn(JohnsIntake.ArmStates.forward)));
    }

    public Command extendHorizontalSLides(){
        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.READY),new Delay(.1).addNext(new MoveHorizontalwithEncoderOut(horizontalSlides,HorizontalSlides.HorizontalSlideStates.Fully_Out,188)).addNext(moveArmJohn(JohnsIntake.ArmStates.forward)));
    }

    public Command bringInHorizontalSLides(){
        return new MultipleCommand(new Delay(.6).addNext(moveClipMag(ClipMech.ArmStates.Out_The_Way)),moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Fully_In));
    }

    public Command bringInHorizontalSLidesBetter(){
        return new MultipleCommand(new MoveHorizontalwithEncoder(this.horizontalSlides,HorizontalSlides.HorizontalSlideStates.Fully_In,180).addNext(moveClipMag(ClipMech.ArmStates.Out_The_Way)),moveArmJohn(JohnsIntake.ArmStates.parallel));
    }

    public Command fullExtendHorizontalSLidesBetter(){
        return new MultipleCommand(new MoveHorizontalwithEncoder(this.horizontalSlides,HorizontalSlides.HorizontalSlideStates.Fully_In,180).addNext(moveClipMag(ClipMech.ArmStates.Out_The_Way)),moveArmJohn(JohnsIntake.ArmStates.parallel));
    }

    public Command moveClipMag(ClipMech.ArmStates armstates){
        return new MoveClipMech(clipmech,armstates);
    }

    public Command moveHorizontalSLides(HorizontalSlides.HorizontalSlideStates slidestates){
        return new MoveHorizontalSlides(horizontalSlides,slidestates);
    }

    public Command movePivotArm(JohnsIntake.ArmStates armstates){
        return new MoveArmJohn(intake,armstates);
    }

    public Command hangJohn(JohnHanging.HangStates hangstates){
        return HangJohn(hangstates);
    }

    public MoveHang HangJohn(JohnHanging.HangStates hangstates){
//        return new MoveHang(hang, hangstates);
        return null;
    }

}