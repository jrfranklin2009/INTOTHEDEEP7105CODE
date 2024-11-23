package org.firstinspires.ftc.teamcode.Robot.robot.Commands.ScoringCommands.SimpleCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.GirlsIntake;



public class MoveArmJohn extends Command {

    ElapsedTime timer = new ElapsedTime();

    GirlsIntake girlsIntake;

    girlsIntake.ArmStates armStates;

    public MoveArmJohn(GirlsIntake johnsIntake, GirlsIntake.ArmStates armStates){
        this.girlsIntake = johnsIntake;
        this.armStates = armStates;
    }
    @Override
    public void init() {
        timer.reset();
        GirlsIntake.setArmStates(armStates);
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return timer.seconds() > 1;
    }

    @Override
    public void shutdown() {

    }
}
