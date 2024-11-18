package org.firstinspires.ftc.teamcode.Robot.robot;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmExtension;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmRotation;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.HangingMechanism.HangingMechanism;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.PinPoint.PinPoint_Odo;

/** sets up the framework for the robt
 */

public class Robot {
    public Dashboard dashboard = new Dashboard();  // set up dashboard
    public Input gamepad1, gamepad2;  // set up gamepads
    public Intake intake;  // set up intake
    public ArmExtension armExtension;  // set up arm
    public ArmRotation armRotation;
    public DriveTrain driveTrain;  // set up the drivetrain
//    public HangingMechanism hanging;  // set up hanging
//    public PinPoint_Odo odo;
    protected CommandScheduler scheduler;  // set up the command scheduler

    public Robot(HardwareMap hw, OpMode opMode, Gamepad gamepad1, Gamepad gamepad2) {
        // init robot
        driveTrain = new DriveTrain(hw);  // drivetrain
//        odo = new PinPoint_Odo();
        armExtension = new ArmExtension();  // arm
        armRotation = new ArmRotation();
        intake= new Intake();  // intake
//        hanging = new HangingMechanism(telemetry);  // hanging
        scheduler = new CommandScheduler(hw,dashboard,intake,driveTrain, armExtension, armRotation);  // set the scheduler up w/ all the subsystems.  MAKE SURE TO ADD NEW SUBSYSTEMS HERE
        this.gamepad1 = new Input(gamepad1,scheduler);
        this.gamepad2 = new Input(gamepad2,scheduler);  // gamepads
        if (opMode.equals(OpMode.Auto)) {  // if auto init the auto
            scheduler.initAuto();
        } else if (opMode.equals(OpMode.Teleop)) {  // if tele init the tele
            scheduler.initTeleop();
        }
    }

    public void update() {  // update everything
        updateGamepads();  // update gamepads
        driveTrain.mecanumDrive.update();  // update mecanum drive
        scheduler.runAuto();  // it says run auto but this really just updates the scheduler and runs everything there
    }

    public void updateTele() {  // update everything but tele specific
        updateGamepads();  // update gamepads
        scheduler.runAuto(); // it says run auto but this really just updates the scheduler and runs everything there
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

