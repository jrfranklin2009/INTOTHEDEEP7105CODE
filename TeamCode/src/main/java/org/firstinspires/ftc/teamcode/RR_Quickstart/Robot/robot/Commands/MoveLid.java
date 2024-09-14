package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LidServo;

public class MoveLid extends Command {

    LidServo lidServo;

    LidServo.LidStates states;

    ElapsedTime timer = new ElapsedTime();

    public MoveLid(LidServo lidServo, LidServo.LidStates states){
        this.lidServo = lidServo;
        this.states = states;
    }

    @Override
    public void init() {
        lidServo.moveLid(states);
        timer.reset();
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return timer.seconds() > .4;
    }

    @Override
    public void shutdown() {

    }
}
