package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.DrivetrainCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain.DriveTrain;

public class UpdateHeadingIMU extends Command {

    DriveTrain driveTrain;

//    ElapsedTime timer = new ElapsedTime();
    boolean resetHeading = false;

    public UpdateHeadingIMU(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }

    @Override
    public void init() {
//            driveTrain.updateHeadingIMU();
            resetHeading = true;
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {
        resetHeading = false;
    }
}
