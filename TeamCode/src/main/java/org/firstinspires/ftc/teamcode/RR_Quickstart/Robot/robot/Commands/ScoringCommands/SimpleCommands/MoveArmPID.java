package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Arm;

public class MoveArmPID extends Command {

    Arm arm;

    Arm.ArmStates armStates;

    public MoveArmPID (Arm arm,Arm.ArmStates armStates){
        this.arm = arm;
        this.armStates = armStates;
    }

    @Override
    public void init() {
        arm.setArmStates(armStates);
    }

    @Override
    public void periodic() {
        arm.setArmStates(armStates);
    }

    @Override
    public boolean completed() {
        return arm.getArmError() < 15;
    }

    @Override
    public void shutdown() {

    }
}