package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;

import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveArmPID;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveHang;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.TurnArmPID;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;

/** This contains all of the commands used in our robot.
 * Most commands are used in tele.
 */

public class ScoringCommandGroups {
    Intake intake;  // intake

    Arm arm;  // scoring mechanism

    HangingMechanism hangingMechanism;  // hanging

    public ScoringCommandGroups( Intake intake, Arm arm, HangingMechanism hangingMechanism) {
        this.intake = intake;
        this.arm = arm;
        this.hangingMechanism = hangingMechanism;

    }

    public Command intakeSample(Input input){ // intake a sample
        return setIntake(Intake.IntakePower.Intake, Intake.Wrist.IntakeSample, Intake.Twist.IntakeSample, input);
    }

    public Command outtakeSample(Input input){  // outtake sample
        return setIntake(Intake.IntakePower.Outtake, Intake.Wrist.OuttakeSample, Intake.Twist.OuttakeSample, input);
    }

    public Command outtakeSpecimen(Input input){  // outtake specimen (place on bar)
        return setIntake(Intake.IntakePower.Stop, Intake.Wrist.PlacingSpecimin, Intake.Twist.PlacingSpecimin, input);
    }

    public Command setIntakeRest(Input input){  // set the intake to its default
        return setIntake(Intake.IntakePower.Stop, Intake.Wrist.Rest, Intake.Twist.Rest, input);
    }
    public Command turnArmWithPID(Arm.TurnStates turnStates){
        return turnArmPID(turnStates);
    }

    public Command moveArmWithPid(Arm.ArmStates armStates){  // move the arm.  Uses a PID controller
        return moveArmPID(armStates);
    }

    public MoveIntake setIntake(Intake.IntakePower intakePower, Intake.Wrist wrist, Intake.Twist twist, Input input){  // set the intake based on inputs
        return new MoveIntake(intake,intakePower, wrist, twist, input);
    }

    public MoveArmPID moveArmPID(Arm.ArmStates armStates){  // move the arm w/ PID.
        return new MoveArmPID(arm,armStates);
    }

    public TurnArmPID turnArmPID(Arm.TurnStates turnStates) {
        return new TurnArmPID(arm, turnStates);
    }

    public Command getReadyToHang(Input input){
        return setHang(HangingMechanism.LeadScrewTurnStates.Hang, HangingMechanism.LeadScrewStates.Down, input);  // get ready to hang by turning the lead screws
    }

    public Command HookOnBar(Input input) {
        return setHang(HangingMechanism.LeadScrewTurnStates.Hang, HangingMechanism.LeadScrewStates.HangFirstLevel, input);  // hook on the first level bar by moving the hooks on the lead screws up
    }

    public Command Hang(Input input){
        return setHang(HangingMechanism.LeadScrewTurnStates.Coast, HangingMechanism.LeadScrewStates.Down, input);  // hang by pulling them back down.  Also set the servos to coast
    }

    public Command ResetHanging(Input input){
        return setHang(HangingMechanism.LeadScrewTurnStates.Normal, HangingMechanism.LeadScrewStates.Down, input);  // reset the hanging in case we do something wrong
    }

    public Command setHang(HangingMechanism.LeadScrewTurnStates leadscrewturnstate, HangingMechanism.LeadScrewStates leadscrewstate, Input input){  // set the hanging position
        return new MoveHang(hangingMechanism, leadscrewturnstate, leadscrewstate, input);
    }

}