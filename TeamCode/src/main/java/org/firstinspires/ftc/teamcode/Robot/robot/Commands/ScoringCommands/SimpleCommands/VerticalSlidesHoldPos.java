package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;

public class VerticalSlidesHoldPos extends Command {

    VerticalSlides verticalslides;

    public VerticalSlidesHoldPos(VerticalSlides verticalslides){
        this.verticalslides = verticalslides;
    }

    @Override
    public void init() {
        verticalslides.getAndSetPower();
    }

    @Override
    public void periodic() {
        verticalslides.getAndSetPower();
    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {

    }
}
