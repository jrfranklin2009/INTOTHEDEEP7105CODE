package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;

public class MoveVerticalSlides extends Command {

    VerticalSlides verticalSlides;

    ElapsedTime time = new ElapsedTime();

//    double ref;

    public MoveVerticalSlides(VerticalSlides verticalSlides){
        this.verticalSlides = verticalSlides;
//        this.ref = ref;
    }

    @Override
    public void init() {
        time.reset();
    }

    @Override
    public void periodic() {
        verticalSlides.pidController();
        if (Math.abs(verticalSlides.getSlidesError()) < 10){

        } else {
            time.reset();
        }
    }

    @Override
    public boolean completed() {
        return Math.abs(verticalSlides.getSlidesError()) < 10 && time.seconds() > 3;
    }

    @Override
    public void shutdown() {

    }
}
