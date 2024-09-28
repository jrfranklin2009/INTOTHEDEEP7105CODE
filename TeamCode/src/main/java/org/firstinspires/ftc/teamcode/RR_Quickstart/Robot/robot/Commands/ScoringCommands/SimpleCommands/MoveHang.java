package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;

public class MoveHang extends Command {

    HangingMechanism hangingMechanism;
    HangingMechanism.LeadScrewStates leadScrewStates;
    HangingMechanism.LeadScrewTurnStates leadScrewTurnStates;
    Input input;
    ElapsedTime timer = new ElapsedTime();

    public MoveHang(HangingMechanism hangingMechanism, HangingMechanism.LeadScrewTurnStates turnStates, HangingMechanism.LeadScrewStates screwStates, Input input) {
        this.hangingMechanism = hangingMechanism;
        this.leadScrewStates = screwStates;
        this.leadScrewTurnStates = turnStates;
        this.input =input;
    }

    @Override
    public void init() {
        hangingMechanism.setLeadScrewTurnStates(leadScrewTurnStates);

        hangingMechanism.setLeadScrewStates(leadScrewStates);
        timer.reset();
    }

    @Override
    public void periodic() {
        hangingMechanism.setLeadScrewStates(leadScrewStates);

    }

    @Override
    public boolean completed() {
        return hangingMechanism.getLeadScrewOneError() < 15 && hangingMechanism.getLeadScrewTwoError() < 15;
    }

    @Override
    public void shutdown() {

    }
}
