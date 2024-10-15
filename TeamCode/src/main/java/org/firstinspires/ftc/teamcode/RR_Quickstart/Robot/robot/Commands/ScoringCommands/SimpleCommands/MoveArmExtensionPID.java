package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.ArmExtension;

public class MoveArmExtensionPID extends Command {

    ArmExtension armExtension;

    ArmExtension.ArmExtensionStates armExtensionStates;

    public MoveArmExtensionPID(ArmExtension armExtension, ArmExtension.ArmExtensionStates armExtensionStates){
        this.armExtension = armExtension;
        this.armExtensionStates = armExtensionStates;

    }

    @Override
    public void init() {
        armExtension.setArmExtensionStates(armExtensionStates);

    }

    @Override
    public void periodic() {
        armExtension.setArmExtensionStates(armExtensionStates);

    }

    @Override
    public boolean completed() {
        return armExtension.getArmExtensionError() < 15;
    }

    @Override
    public void shutdown() {

    }
}