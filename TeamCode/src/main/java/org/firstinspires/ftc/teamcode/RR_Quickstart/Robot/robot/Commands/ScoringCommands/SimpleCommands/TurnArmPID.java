package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm;

public class TurnArmPID extends Command {

    Arm arm;
    Arm.TurnStates turnStates;
    public TurnArmPID (Arm arm, Arm.TurnStates turnStates){
        this.arm = arm;
        this.turnStates = turnStates;
    }

    @Override
    public void init() {
        arm.setRotateStates(turnStates);
    }

    @Override
    public void periodic() {
        arm.setRotateStates(turnStates);
    }

    @Override
    public boolean completed() {
        return arm.getRotateError() < 15;
    }

    @Override
    public void shutdown() {

    }
}
