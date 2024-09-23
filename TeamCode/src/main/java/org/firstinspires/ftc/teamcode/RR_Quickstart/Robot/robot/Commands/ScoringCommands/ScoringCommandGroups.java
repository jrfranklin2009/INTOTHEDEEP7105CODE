package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;

import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveArmPID;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;

public class ScoringCommandGroups {
    Intake intake;

    Arm arm;

    public ScoringCommandGroups( Intake intake, Arm arm) {
        this.intake = intake;
        this.arm = arm;
    }

    public Command intakeSample(Input input){
        return setIntake(Intake.IntakePower.Intake, Intake.Wrist.IntakeSample, Intake.Twist.IntakeSample, input);
    }

    public Command outtakeSample(Input input){
        return setIntake(Intake.IntakePower.Outtake, Intake.Wrist.OuttakeSample, Intake.Twist.OuttakeSample, input);
    }

    public Command outtakeSpecimen(Input input){
        return setIntake(Intake.IntakePower.Stop, Intake.Wrist.PlacingSpecimin, Intake.Twist.PlacingSpecimin, input);
    }

    public Command setIntakeRest(Input input){
        return setIntake(Intake.IntakePower.Stop, Intake.Wrist.Rest, Intake.Twist.Rest, input);
    }

    public Command moveArmWithPid(Arm.ArmStates armStates){
        return moveArmPID(armStates);
    }

    public MoveIntake setIntake(Intake.IntakePower intakePower, Intake.Wrist wrist, Intake.Twist twist, Input input){
        return new MoveIntake(intake,intakePower, wrist, twist, input);
    }

    public MoveArmPID moveArmPID(Arm.ArmStates armStates){
        return new MoveArmPID(arm,armStates);
    }

}