package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;

public class CloseThread extends Command {
    VerticalSlides verticalslides;

    ElapsedTime time = new ElapsedTime();

    boolean secondLoop = false;

    public CloseThread(VerticalSlides verticalslides){
        this.verticalslides = verticalslides;
    }

    @Override
    public void init() {
        time.reset();
        secondLoop = false;
    }

    @Override
    public void periodic() {
//        verticalslides.getAndSetPower();
        //TODO: Check to see if we can remove the timer
        if (time.seconds() > .8 && secondLoop == false){
            verticalslides.closeSLIDEThread();
            secondLoop = true;
//        verticalslides.getAndSetPower();
        }
    }

    @Override
    public boolean completed() {
        return secondLoop == true;
    }

    @Override
    public void shutdown() {
//        verticalslides.zeroPower();
    }
}
