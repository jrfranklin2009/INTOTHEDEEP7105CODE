package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveTrain.PlaneShooter;

public class ShootPlane extends Command {

    PlaneShooter planeShooter;

    ElapsedTime timer = new ElapsedTime();

    public ShootPlane(PlaneShooter planeShooter){
        this.planeShooter = planeShooter;
    }

    @Override
    public void init() {
        planeShooter.shootPlane();
        timer.reset();
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
