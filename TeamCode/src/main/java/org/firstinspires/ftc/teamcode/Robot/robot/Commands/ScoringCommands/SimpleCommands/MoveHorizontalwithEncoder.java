package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.HorizontalSlides;

public class MoveHorizontalwithEncoder extends Command {

    HorizontalSlides horizontalslides;

    HorizontalSlides.HorizontalSlideStates horizontalslidestates;

    double target;

    public MoveHorizontalwithEncoder(HorizontalSlides horizontalslides, HorizontalSlides.HorizontalSlideStates horizontalslidestates, double target){
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
        return horizontalslides.getSlidePos() <= target;
    }

    @Override
    public void shutdown() {

    }
}

