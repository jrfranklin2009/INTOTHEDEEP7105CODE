package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;

public class MoveClipMag extends Command {

    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return time.seconds() > 1;
    }

    @Override
    public void shutdown() {

    }
}
