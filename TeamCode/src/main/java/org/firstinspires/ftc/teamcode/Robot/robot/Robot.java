package org.firstinspires.ftc.teamcode.Robot.robot;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFrameWork.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmExtension;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DepositingMechanisms.ArmRotation;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake.GirlsIntake;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.HangingMechanism.ViperSlidesHanging;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.LimeLight;

/** sets up the framework for the robt
 */

public class Robot {
    public Dashboard dashboard = new Dashboard();  // set up dashboard
    public Input gamepad1, gamepad2;  // set up gamepads
    public GirlsIntake intake;  // set up intake
    public DriveTrain driveTrain;  // set up the drivetrain
    public ViperSlidesHanging hanging; // set up the hanging
    public ArmRotation rotation;
    public ArmExtension extention;
    protected CommandScheduler scheduler;  // set up the command scheduler
    public LimeLight limelight;


    public Robot(HardwareMap hw, OpMode opMode, Gamepad gamepad1, Gamepad gamepad2) {
        // init robot
        driveTrain = new DriveTrain(hw);  // drivetrain
        limelight = new LimeLight(driveTrain);
        intake= new GirlsIntake();// intake
        hanging= new ViperSlidesHanging(); // hanging
        rotation= new ArmRotation();
        extention= new ArmExtension();
        scheduler = new CommandScheduler(hw,dashboard,intake,driveTrain,extention,rotation,hanging,limelight);  // set the scheduler up w/ all the subsystems.  MAKE SURE TO ADD NEW SUBSYSTEMS HERE
        this.gamepad1 = new Input(gamepad1,scheduler);
        this.gamepad2 = new Input(gamepad2,scheduler);
        // gamepads
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

