package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;



public class MoveIntake extends Command {

    Intake intake;

    Intake.Wrist intakeWristStates;

    Intake.IntakePower intakeStates;

    Input input;
    ElapsedTime timer = new ElapsedTime();

    public MoveIntake(Intake intake, Intake.IntakePower intakeStates, Intake.Wrist intakeWristStates, Input input){
        this.intake = intake;
        this.intakeStates = intakeStates;
        this.intakeWristStates = intakeWristStates;
        this.input = input;
    }

    @Override
    public void init() {
        intake.setIntakePower(intakeStates);
        timer.reset();
        intake.setWrist(intakeWristStates);
    }

    @Override
    public void periodic() {

    }
    // ||
    @Override
    public boolean completed() {
        return !input.isSquarePressed() && !input.isRight_bumper() && !input.isLeft_bumper();
    }

    @Override
    public void shutdown() {
        intake.setIntakePower(Intake.IntakePower.Stop);
    }
}
