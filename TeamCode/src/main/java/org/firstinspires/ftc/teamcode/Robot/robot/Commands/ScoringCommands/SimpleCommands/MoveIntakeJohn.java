package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.JohnsIntake;


public class MoveIntakeJohn extends Command {

    Input input;

    JohnsIntake johnsIntake;

    HorizontalSlides slides;

    public MoveIntakeJohn(Input input, JohnsIntake johnsIntake, HorizontalSlides slides){
        this.input = input;
        this.johnsIntake = johnsIntake;
        this.slides = slides;
    }
    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        if (input.isLeft_trigger_press()){
            johnsIntake.setIntake(JohnsIntake.IntakeStates.outtake);
        } else if (input.isRight_trigger_press()) {
            johnsIntake.setIntake(JohnsIntake.IntakeStates.intake);
            johnsIntake.setArmStates(JohnsIntake.ArmStates.forward);
        } else if (slides.leftservoslide.getPosition() != .24
                && !input.isRight_trigger_press() && !input.isLeft_trigger_press()) {
            johnsIntake.setArmStates(JohnsIntake.ArmStates.parallel);
            johnsIntake.setIntake(JohnsIntake.IntakeStates.stop);
        } else {
            johnsIntake.setIntake(JohnsIntake.IntakeStates.stop);
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