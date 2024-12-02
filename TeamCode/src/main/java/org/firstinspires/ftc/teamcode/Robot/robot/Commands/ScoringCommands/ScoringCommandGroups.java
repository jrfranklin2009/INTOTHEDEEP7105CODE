package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands;


import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;

import org.firstinspires.ftc.teamcode.CommandFrameWork.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveArmJohn;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveGripper;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHorizontalwithEncoder;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntakeJohn;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveVerticalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.ClipMech;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmExtension;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmRotation;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveArmExtensionPID;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveArmRotation;
import org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntake;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;
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

    HorizontalSlides horizontalSlides;

    VerticalSlides verticalslides;

    public ScoringCommandGroups(JohnsIntake intake, VerticalSlides verticalslides, HorizontalSlides horizontalslides,ClipMech clipmech) {
        this.intake = intake;
        this.verticalslides = verticalslides;
        this.horizontalSlides = horizontalslides;
        this.clipmech = clipmech;
//        this.armExtension = armExtension;
//        this.armRotation = armRotation;
//        this.hangingMechanism = hangingMechanism;

    }

    public Command armOutBack(){
        return new MultipleCommand(moveArmJohn(JohnsIntake.ArmStates.outback),moveGripper(JohnsIntake.GripperStates.unclamp));
    }

    public Command moveArmJohn(JohnsIntake.ArmStates armStates){
        return new MoveArmJohn(this.intake, armStates);
    }

    public Command moveIntakeJohn(Input input){
        return new MoveIntakeJohn(input,this.intake);
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

    public Command extendHorizontalSLides(){
        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.READY),new Delay(.1).addNext(moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Half_Out)),new Delay(.3).addNext(moveArmJohn(JohnsIntake.ArmStates.forward)));
    }

    public Command fullExtendHorizontalSLides(){
        return new MultipleCommand(moveClipMag(ClipMech.ArmStates.READY),new Delay(.1).addNext(moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Fully_Out)),new Delay(.3).addNext(moveArmJohn(JohnsIntake.ArmStates.forward)));
    }

    public Command bringInHorizontalSLides(){
        return new MultipleCommand(new Delay(.6).addNext(moveClipMag(ClipMech.ArmStates.Out_The_Way)),moveHorizontalSlides(HorizontalSlides.HorizontalSlideStates.Fully_In));
    }

    public Command bringInHorizontalSLidesBetter(){
        return new MoveHorizontalwithEncoder(this.horizontalSlides,HorizontalSlides.HorizontalSlideStates.Fully_In,147).addNext(moveClipMag(ClipMech.ArmStates.Out_The_Way));
    }

//    public Command autoCLIP(){
//        return moveClipMag(ClipMech.ArmStates.READY).addNext();
//    }

    public Command moveClipMag(ClipMech.ArmStates armstates){
        return new MoveClipMech(clipmech,armstates);
    }

    public Command moveHorizontalSLides(HorizontalSlides.HorizontalSlideStates slidestates){
        return new MoveHorizontalSlides(horizontalSlides,slidestates);
    }

    public Command movePivotArm(JohnsIntake.ArmStates armstates){
        return new MoveArmJohn(intake,armstates);
    }

//
//    public Command outtakeSample(Input input){  // outtake sample
//        return setIntake(Intake.IntakePower.Outtake, Intake.Wrist.OuttakeSample, Intake.Twist.OuttakeSample, input);
//    }
//
//    public Command outtakeSpecimen(Input input){  // outtake specimen (place on bar)
//        return setIntake(Intake.IntakePower.Stop, Intake.Wrist.PlacingSpecimin, Intake.Twist.PlacingSpecimin, input);
//    }
//
//    public Command setIntakeRest(Input input){  // set the intake to its default
//        return setIntake(Intake.IntakePower.Stop, Intake.Wrist.Rest, Intake.Twist.Rest, input);
//    }


//    public MoveIntake setIntake(Intake.IntakePower intakePower, Intake.Wrist wrist, Intake.Twist twist, Input input){  // set the intake based on inputs
//        return new MoveIntake(intake,intakePower, wrist, twist, input);
//    }

//    public MoveArmExtensionPID moveArmExtensionPID(ArmExtension.ArmExtensionStates armExtension){  // move the arm w/ PID.
//        return new MoveArmExtensionPID(this.armExtension,armExtension);
//    }
//
//    public MoveArmRotation moveArmRotationPID(ArmRotation.ArmRotationStates turnStates) {
//        return new MoveArmRotation(this.armRotation, turnStates);
//    }

//    public Command moveJohn

//    public Command getReadyToHang(Input input){
//        return setHang(HangingMechanism.LeadScrewTurnStates.Hang, HangingMechanism.LeadScrewStates.Down, input);  // get ready to hang by turning the lead screws
//    }
//
//    public Command HookOnBar(Input input) {
//        return setHang(HangingMechanism.LeadScrewTurnStates.Hang, HangingMechanism.LeadScrewStates.HangFirstLevel, input);  // hook on the first level bar by moving the hooks on the lead screws up
//    }
//
//    public Command Hang(Input input){
//        return setHang(HangingMechanism.LeadScrewTurnStates.Coast, HangingMechanism.LeadScrewStates.Down, input);  // hang by pulling them back down.  Also set the servos to coast
//    }
//
//    public Command ResetHanging(Input input){
//        return setHang(HangingMechanism.LeadScrewTurnStates.Normal, HangingMechanism.LeadScrewStates.Down, input);  // reset the hanging in case we do something wrong
//    }

//    public Command setHang(HangingMechanism.LeadScrewTurnStates leadscrewturnstate, HangingMechanism.LeadScrewStates leadscrewstate, Input input){  // set the hanging position
//        return new MoveHang(hangingMechanism, leadscrewturnstate, leadscrewstate, input);
//    }

}