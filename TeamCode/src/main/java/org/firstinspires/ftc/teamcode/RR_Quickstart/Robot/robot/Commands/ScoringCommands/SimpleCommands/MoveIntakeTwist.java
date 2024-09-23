package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;

public class MoveIntakeTwist extends Command {

    Intake.Twist twist;
    Intake intake;
    ElapsedTime timer = new ElapsedTime();

    public MoveIntakeTwist(Intake intake, Intake.Twist twist, Input input){
        this.twist = twist;
        this.intake = intake;
    }

    @Override
    public void init() {
        intake.setTwist(twist);
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean completed() {
        return (timer.seconds() > 5);
    }

    @Override
    public void shutdown() {

    }
}
