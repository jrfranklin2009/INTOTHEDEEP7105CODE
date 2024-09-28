package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;

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
