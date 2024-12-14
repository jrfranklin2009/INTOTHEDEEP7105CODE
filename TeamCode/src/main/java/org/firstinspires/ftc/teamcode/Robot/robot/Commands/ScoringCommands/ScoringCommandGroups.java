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
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHorizontalSlidesEncoder;
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
    ClipMech clipmech;
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

    public Command initRobot(){
        return new MultipleCommand(moveGripper(JohnsIntake.GripperStates.clamp),moveArmJohn(JohnsIntake.ArmStates.preauto_clip));
    }

    public Command slidesTeleop(){
//        return new MoveVerticalSlides(verticalslides);
        return null;
    }

//    public Command slidesTeleop(){
//        return new MoveVerticalSlidesMultiThread(verticalslides,opMode,false,0).addNext(new CloseThread(verticalslides)).addNext(new VerticalSlidesHoldPos(verticalslides));
//    }

    public Command slidesSetPos(double target){
        return new MoveVerticalSlidesMultiThread(verticalslides,opMode,true,target).addNext(new CloseThread(verticalslides)).addNext(new VerticalSlidesHoldPos(verticalslides));
    }

    public Command moveClipMechanismsOut(double verticalSlidesTarget, ClipMech.ArmStates clipstate, HorizontalSlides.HorizontalSlideStates horizontalstate, double target, JohnsIntake.ArmStates armstate){
        return new MultipleCommand(slidesSetPos(verticalSlidesTarget),extendHorizontalSlides_VerticalSlides(clipstate,horizontalstate,target,armstate));
    }

    public Command armOutBack(){
        return new MultipleCommand(moveArmJohn(JohnsIntake.ArmStates.outback),moveGripper(JohnsIntake.GripperStates.unclamp));
    }

    public Command armOutFront(){
        return new MultipleCommand(moveArmJohn(JohnsIntake.ArmStates.forward),moveGripper(JohnsIntake.GripperStates.unclamp));
    }


    public Command moveArmJohn(JohnsIntake.ArmStates armStates){
        return new MoveArmJohn(this.intake, armStates);
    }

    public Command moveGripper(JohnsIntake.GripperStates gripperStates){
        return new MoveGripper(this.intake, gripperStates);
    }

    public Command moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates horizontalslidestates, double target){
        return new MoveHorizontalSlidesEncoder(this.horizontalSlides,horizontalslidestates,target);
    }

    public Command fullExtendHorizontalSLides(){
        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.READY),
                new Delay(.1).addNext(moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Fully_Out,245)));
//                                .addNext(moveArmJohn(JohnsIntake.ArmStates.forward)));
    }

    public Command clipClip(){
        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.Out_The_Way),moveArmJohn(JohnsIntake.ArmStates.preauto_clip),new Delay(.1).addNext(moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Half_Out,190)).addNext(moveArmJohn(JohnsIntake.ArmStates.posauto_clip).addNext(moveGripper(JohnsIntake.GripperStates.unclamp))));
    }

    public Command extendHorizontalSLides(){
        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.READY)
                ,new Delay(.1).addNext(moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Half_Out,190)));
//                .addNext(moveArmJohn(JohnsIntake.ArmStates.forward)));
    }

    public Command extendHorizontalSlides_VerticalSlides(ClipMech.ArmStates clipstate, HorizontalSlides.HorizontalSlideStates horizontalstate, double target, JohnsIntake.ArmStates armstate){
        return new MultipleCommand(moveClipMag(clipstate),new Delay(.4).addNext(moveHorizontalSlides(horizontalstate,target)).addNext(moveArmJohn(armstate)));
    }

    public Command bringInHorizontalSLidesBetter(){
        return new MultipleCommand(new MoveHorizontalSlidesEncoder(this.horizontalSlides,HorizontalSlides.HorizontalSlideStates.Fully_In,170).addNext(moveClipMag(ClipMech.ArmStates.Out_The_Way)),moveArmJohn(JohnsIntake.ArmStates.parallel));
    }

    public Command moveClipMag(ClipMech.ArmStates armstates){
        return new MoveClipMech(clipmech,armstates);
    }

    public Command hangJohn(JohnHanging.HangStates hangstates){
        return HangJohn(hangstates);
    }

    public MoveHang HangJohn(JohnHanging.HangStates hangstates){
//        return new MoveHang(hang, hangstates);
        return null;
    }

}