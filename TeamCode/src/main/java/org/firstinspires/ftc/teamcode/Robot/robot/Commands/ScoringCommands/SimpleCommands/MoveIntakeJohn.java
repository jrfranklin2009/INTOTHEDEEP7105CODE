package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;


public class MoveIntakeJohn extends Command {

    ElapsedTime timer = new ElapsedTime();

    Input input;

    JohnsIntake johnsIntake;

    JohnsIntake.IntakeStates intakeStates;

    public MoveIntakeJohn(Input input, JohnsIntake johnsIntake, JohnsIntake.IntakeStates intakeStates){
      this.input = input;
      this.johnsIntake = johnsIntake;
      this.intakeStates = intakeStates;
    }
    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return !input.isRightTriggerPressed() || !input.isLeftTriggerPressed();
    }

    @Override
    public void shutdown() {
        intakeStates = JohnsIntake.IntakeStates.stop;
    }
}
