package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.GirlsIntake;


public class MoveIntakeGirls extends Command {

    Input input;

    GirlsIntake GirlsIntake;

    public MoveIntakeGirls(Input input, Input GirlsIntake){
      this.input = input;
      this.GirlsIntake = GirlsIntake;
    }
    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        if (input.isLeft_trigger_press()){
            GirlsIntake.setIntake(org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.GirlsIntake.IntakeStates.outtake);
        } else if (input.isRight_trigger_press()) {
            GirlsIntake.setIntake(org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.GirlsIntake.IntakeStates.intake);
        }else {
            GirlsIntake.setIntake(org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.GirlsIntake.IntakeStates.stop);
        }
    }

    @Override
    public boolean completed() {
        return !input.isRight_trigger_press() || !input.isLeft_trigger_press();
    }

    @Override
    public void shutdown() {
    }
}
