package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.VerticalSlides;

public class MoveVerticalSlidesMultiThread extends Command {

    VerticalSlides verticalSlides;

    ElapsedTime time = new ElapsedTime();

    LinearOpMode opMode;

    boolean setTarget;

    double ref;

    public MoveVerticalSlidesMultiThread(VerticalSlides verticalSlides, LinearOpMode opMode, boolean setTarget, double ref){
        this.verticalSlides = verticalSlides;
        this.opMode = opMode;
        this.setTarget = setTarget;
        this.ref = ref;
    }

    @Override
    public void init() {
        if (setTarget){
            VerticalSlides.ref = ref;
        }
        time.reset();
//        if (rat == true){
//        verticalSlides.startSLIDEThread();
//        rat = false;}
//        VerticalSlides.closeThread = false;
    }

    @Override
    public void periodic() {
        verticalSlides.pidController();
//        if (Math.abs(verticalSlides.getSlidesError()) < 20){
//
//        }
//        else if (verticalSlides.isThreadInterrupted()) {
//            verticalSlides.closeSLIDEThread();
//        }
//        else {
//            time.reset();
//        }
    }

    @Override
    public boolean completed() {
        return Math.abs(verticalSlides.getSlidesError()) < 10;
    }

    @Override
    public void shutdown() {
//        verticalSlides.closeSLIDEThread();
//        new VerticalSlidesHoldPos(verticalSlides);
//        verticalSlides.closeSLIDEThread();
    }
}
