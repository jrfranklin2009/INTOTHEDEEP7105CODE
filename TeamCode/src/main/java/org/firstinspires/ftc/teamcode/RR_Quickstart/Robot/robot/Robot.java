package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Slides;

public class Robot {
    public Dashboard dashboard = new Dashboard();
    public Input gamepad1, gamepad2;
    public Intake intake = new Intake();
    public Slides slides = new Slides();
    public DriveTrain driveTrain;
    protected CommandScheduler scheduler;

    public Robot(HardwareMap hw, OpMode opMode, Gamepad gamepad1, Gamepad gamepad2) {
        driveTrain = new DriveTrain(hw);
        scheduler = new CommandScheduler(hw,dashboard,intake, slides,driveTrain);
        this.gamepad1 = new Input(gamepad1,scheduler);
        this.gamepad2 = new Input(gamepad2,scheduler);
        if (opMode.equals(OpMode.Auto)) {
            scheduler.initAuto();
        } else if (opMode.equals(OpMode.Teleop)) {
            scheduler.initTeleop();
        }
    }
    public void update() {
        updateGamepads();
        driveTrain.mecanumDrive.update();
        scheduler.runAuto();
    }

    public void updateTele() {
        updateGamepads();
        scheduler.runAuto();
    }

    public void shutdown() {
        scheduler.shutdown();
    }

    public CommandScheduler getScheduler() {
        return scheduler;
    }

    public void updateGamepads(){
        gamepad1.periodicAuto();
        gamepad2.periodicAuto();
    }

    public enum OpMode {
        Auto,
        Teleop
    }
}

