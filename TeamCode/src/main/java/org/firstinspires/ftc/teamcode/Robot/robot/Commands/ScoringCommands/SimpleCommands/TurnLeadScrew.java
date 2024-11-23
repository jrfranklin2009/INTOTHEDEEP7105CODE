package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;

public class TurnLeadScrew extends Command {
    HangingMechanism hang;
    HangingMechanism.LeadScrewTurnStates turnStates;
    ElapsedTime timer =new ElapsedTime();

    public TurnLeadScrew(HangingMechanism hang, HangingMechanism.LeadScrewTurnStates turnStates, Input input) {
        this.hang = hang;
        this.turnStates = turnStates;
    }

    @Override
    public void init() {
        hang.setLeadScrewTurnStates(HangingMechanism.LeadScrewTurnStates.Normal);

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
