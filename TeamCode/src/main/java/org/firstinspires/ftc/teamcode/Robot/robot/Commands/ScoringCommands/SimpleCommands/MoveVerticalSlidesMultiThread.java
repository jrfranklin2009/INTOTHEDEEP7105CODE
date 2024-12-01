package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;

public class MoveVerticalSlidesMultiThread extends Command {

    VerticalSlides verticalSlides;

    ElapsedTime time = new ElapsedTime();

    LinearOpMode opMode;

//    double ref;

    public MoveVerticalSlidesMultiThread(VerticalSlides verticalSlides, LinearOpMode opMode){
        this.verticalSlides = verticalSlides;
        this.opMode = opMode;
//        this.ref = ref;
    }

    @Override
    public void init() {
        time.reset();
        verticalSlides.startSLIDEThread(opMode);
    }

    @Override
    public void periodic() {
        if (Math.abs(verticalSlides.getSlidesError()) < 10){

        }
        else if (verticalSlides.isThreadInterrupted()) {
            verticalSlides.closeSLIDEThread();
        }
        else {
            time.reset();
        }
    }

    @Override
    public boolean completed() {
        return Math.abs(verticalSlides.getSlidesError()) < 10 && time.seconds() > 3;
    }

    @Override
    public void shutdown() {
//        verticalSlides.closeSLIDEThread();
    }
}
