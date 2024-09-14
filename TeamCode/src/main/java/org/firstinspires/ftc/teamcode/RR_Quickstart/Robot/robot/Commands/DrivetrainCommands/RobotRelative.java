package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands;


import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DriveTrain.DriveTrain;

public class RobotRelative extends Command {

    DriveTrain driveTrain;

    Input input;

    public RobotRelative(DriveTrain driveTrain, Input input){
      this.driveTrain = driveTrain;
      this.input = input;
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        driveTrain.RobotRelative(input);
    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
        driveTrain.RobotRelative(input);
    }
}
