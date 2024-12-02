package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.HangingMechanism.JohnHanging;

public class MoveHang extends Command {
    JohnHanging hanging;
    JohnHanging.HangStates hangStates;
    ElapsedTime timer = new ElapsedTime();

    public MoveHang(JohnHanging hanging, JohnHanging.HangStates hangStates){
        this.hanging = hanging;
        this.hangStates = hangStates;


    }

    @Override
    public void init() {
        hanging.setRightHang(hangStates);
        timer.reset();
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return (timer.seconds() > 5.0);
    }

    @Override
    public void shutdown() {

    }
}
