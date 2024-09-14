package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;

import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands.MoveIntake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Slides;

public class ScoringCommandGroups {

    Slides slides;
    Intake intake;

    public ScoringCommandGroups(Slides slides, Intake intake) {
        this.slides = slides;
        this.intake = intake;
    }

    public Command intakeSample(Input input){
        return setIntake(Intake.IntakePower.Intake, Intake.Wrist.Sample, input);
    }

    public Command outtakeSample(Input input){
        return setIntake(Intake.IntakePower.Outtake, Intake.Wrist.Sample, input);
    }

    public Command outtakeSpecimen(Input input){
        return setIntake(Intake.IntakePower.Stop, Intake.Wrist.PlacingSpecimin, input);
    }

    public MoveIntake setIntake(Intake.IntakePower intakePower, Intake.Wrist wrist, Input input){
        return new MoveIntake(intake,intakePower, wrist, input);
    }

}