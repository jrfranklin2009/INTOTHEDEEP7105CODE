package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;

public class VerticalSlidesHoldPos extends Command {

    VerticalSlides verticalslides;

//    ElapsedTime time = new ElapsedTime();

//    boolean secondLoop = false;

    public VerticalSlidesHoldPos(VerticalSlides verticalslides){
        this.verticalslides = verticalslides;
    }

    @Override
    public void init() {
//       time.reset();
//       secondLoop = false;
        verticalslides.getAndSetPower();
    }

    @Override
    public void periodic() {
//        verticalslides.getAndSetPower();
//        if (time.seconds() > 2 && secondLoop == false){
//            verticalslides.closeSLIDEThread();
//            secondLoop = true;
//        verticalslides.getAndSetPower();

            verticalslides.getAndSetPower();
    }

    @Override
    public boolean completed() {
        return verticalslides.ref == 0;
    }

    @Override
    public void shutdown() {

    }
}
