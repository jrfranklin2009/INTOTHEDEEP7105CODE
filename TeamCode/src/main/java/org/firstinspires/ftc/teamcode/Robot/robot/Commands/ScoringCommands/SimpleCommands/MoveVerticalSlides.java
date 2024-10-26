package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;

public class MoveVerticalSlides extends Command {

    VerticalSlides verticalSlides;

    double ref;

    public MoveVerticalSlides(VerticalSlides verticalSlides, double ref){
        this.verticalSlides = verticalSlides;
        this.ref = ref;
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        verticalSlides.pidController(ref);
    }

    @Override
    public boolean completed() {
        return verticalSlides.getSlidesError(ref) > 10;
    }

    @Override
    public void shutdown() {

    }
}
