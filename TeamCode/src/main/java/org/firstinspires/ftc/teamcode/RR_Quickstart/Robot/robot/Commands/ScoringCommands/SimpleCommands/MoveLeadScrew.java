package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;

public class MoveLeadScrew extends Command {

    HangingMechanism.LeadScrewStates leadScrew;
    HangingMechanism hang;
    ElapsedTime timer =new ElapsedTime();

    public MoveLeadScrew(HangingMechanism hang, HangingMechanism.LeadScrewStates leadScrew, Input input ){
        this.hang = hang;
        this.leadScrew =leadScrew;
    }

    @Override
    public void init() {
//        hang.setLeadScrewStates(HangingMechanism.LeadScrewStates.Down);
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return (timer.seconds() < 5);
    }

    @Override
    public void shutdown() {

    }
}
