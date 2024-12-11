package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;

public class MoveHorizontalwithEncoderOut extends Command {

    HorizontalSlides horizontalslides;

    HorizontalSlides.HorizontalSlideStates horizontalslidestates;

    double target;

    public MoveHorizontalwithEncoderOut(HorizontalSlides horizontalslides, HorizontalSlides.HorizontalSlideStates horizontalslidestates, double target){
        this.horizontalslides = horizontalslides;
        this.horizontalslidestates = horizontalslidestates;
        this.target = target;
    }

    @Override
    public void init() {
        horizontalslides.setHorizontalSlides(horizontalslidestates);
//        horizontalslides.setPosition(target);
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return horizontalslides.getSlidePos() >= target;
    }

    @Override
    public void shutdown() {

    }
}